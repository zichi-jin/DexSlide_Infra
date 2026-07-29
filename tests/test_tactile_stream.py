import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from tactile_stream import (
    RECORD_DATA,
    RECORD_STATUS,
    SYSTEM_PORT_ID,
    TactileStreamMonitor,
    TactileRecordParser,
    build_sensor_frame,
    crc16_ccitt,
    crc16_modbus,
    decode_status,
    encode_record,
    format_statistics,
)


def test_crc_reference_vectors_match_both_protocol_layers():
    assert crc16_ccitt(b"123456789") == 0x29B1
    assert crc16_modbus(b"123456789") == 0x4B37


def test_parser_recovers_exact_payload_across_arbitrary_chunks():
    payload = bytes(range(256)) + b"\xD3\x5C" + bytes(range(254))
    encoded = encode_record(RECORD_DATA, 1, payload)
    parser = TactileRecordParser()

    records = []
    for chunk in (encoded[:1], encoded[1:9], encoded[9:333], encoded[333:]):
        records.extend(parser.feed(chunk))

    assert len(records) == 1
    assert records[0].port_id == 1
    assert records[0].payload == payload


def test_parser_discards_bad_crc_and_resynchronizes():
    bad = bytearray(encode_record(RECORD_DATA, 0, b"bad"))
    bad[-1] ^= 0xFF
    good = encode_record(RECORD_DATA, 10, b"\xA5\x01\x22\x00\x00\x00\x00\x5A")
    parser = TactileRecordParser()

    records = parser.feed(b"noise" + bytes(bad) + good)

    assert parser.bad_records == 1
    assert len(records) == 1
    assert records[0].port_id == 10
    assert records[0].payload == b"\xA5\x01\x22\x00\x00\x00\x00\x5A"


def test_status_payload_decodes_per_port_counters():
    payload = bytearray(96)
    payload[0] = 1
    payload[1] = 11
    payload[4 + 4 * 3:8 + 4 * 3] = (7).to_bytes(4, "little")
    payload[48 + 4 * 9:52 + 4 * 9] = (2).to_bytes(4, "little")
    payload[92:96] = (8192).to_bytes(4, "little")
    record = encode_record(RECORD_STATUS, SYSTEM_PORT_ID, bytes(payload))
    parser = TactileRecordParser()

    records = parser.feed(record)
    status = decode_status(records[0].payload)

    assert status.overflow[3] == 7
    assert status.uart_errors[9] == 2
    assert status.version == 1
    assert status.rx_halves[3] is None
    assert status.usb_tx_high_water == 8192


def test_status_v2_decodes_rx_activity_and_last_uart_errors():
    payload = bytearray(184)
    payload[0] = 2
    payload[1] = 11
    payload[92 + 4 * 1:96 + 4 * 1] = (200).to_bytes(4, "little")
    payload[136 + 4 * 1:140 + 4 * 1] = (0x0C).to_bytes(4, "little")
    payload[180:184] = (4096).to_bytes(4, "little")
    parser = TactileRecordParser()

    records = parser.feed(encode_record(RECORD_STATUS, SYSTEM_PORT_ID, bytes(payload)))
    status = decode_status(records[0].payload)

    assert status.version == 2
    assert status.rx_halves[1] == 200
    assert status.last_uart_errors[1] == 0x0C
    assert status.usb_tx_high_water == 4096


def test_status_v3_decodes_last_dma_error():
    payload = bytearray(228)
    payload[0] = 3
    payload[1] = 11
    payload[136 + 4 * 4:140 + 4 * 4] = (0x10).to_bytes(4, "little")
    payload[180:184] = (1044).to_bytes(4, "little")
    payload[184 + 4 * 4:188 + 4 * 4] = (0x02).to_bytes(4, "little")

    status = decode_status(payload)

    assert status.version == 3
    assert status.last_uart_errors[4] == 0x10
    assert status.usb_tx_high_water == 1044
    assert status.last_dma_errors[4] == 0x02


def test_status_v4_decodes_current_usb_pending_bytes():
    payload = bytearray(236)
    payload[0] = 4
    payload[1] = 11
    payload[180:184] = (32768).to_bytes(4, "little")
    payload[228:232] = (1234).to_bytes(4, "little")
    payload[232:236] = (5).to_bytes(4, "little")

    status = decode_status(payload)

    assert status.version == 4
    assert status.usb_tx_high_water == 32768
    assert status.usb_tx_pending == 1234
    assert status.status_drops == 5


def test_statistics_reports_mcu_record_crc_errors():
    monitor = TactileStreamMonitor()

    report = format_statistics(monitor, 1.0, 3)

    assert report == "record_bad=3"


def test_monitor_reassembles_combined_frame_and_counts_sequence_gap():
    data_1 = (1234).to_bytes(4, "little") + bytes([12, 8]) + (17).to_bytes(2, "little") + bytes(12 * 8 * 6)
    data_2 = (1244).to_bytes(4, "little") + bytes([12, 8]) + (20).to_bytes(2, "little") + bytes(12 * 8 * 6)
    frame_1 = build_sensor_frame(1, 0x22, data_1)
    frame_2 = build_sensor_frame(1, 0x22, data_2)
    assert len(frame_1) == 592

    stream = frame_1 + frame_2
    records = [
        encode_record(RECORD_DATA, 1, stream[:512]),
        encode_record(RECORD_DATA, 1, stream[512:]),
    ]
    record_parser = TactileRecordParser()
    monitor = TactileStreamMonitor()
    frames = []

    for encoded in records:
        for record in record_parser.feed(encoded):
            frames.extend(monitor.feed_record(record))

    assert [frame.raw for frame in frames] == [frame_1, frame_2]
    assert monitor.statistics[1].seq_lost == 2
    assert monitor.statistics[1].sensor_length_errors == 0


def test_eleven_port_combined_stream_survives_dma_and_usb_fragmentation():
    def combined_frame(port_id: int, seq: int) -> bytes:
        data = (
            (seq * 10).to_bytes(4, "little")
            + bytes([12, 8])
            + seq.to_bytes(2, "little")
            + bytes([port_id]) * (12 * 8 * 6)
        )
        return build_sensor_frame(1, 0x22, data)

    port_records = {}
    for port_id in range(11):
        stream = b"".join(combined_frame(port_id, seq) for seq in range(100))
        port_records[port_id] = [
            encode_record(RECORD_DATA, port_id, stream[offset:offset + 512])
            for offset in range(0, len(stream), 512)
        ]

    transport = bytearray()
    for record_index in range(max(len(records) for records in port_records.values())):
        for port_id in range(11):
            records = port_records[port_id]
            if record_index < len(records):
                transport.extend(records[record_index])

    record_parser = TactileRecordParser()
    monitor = TactileStreamMonitor()
    chunk_sizes = [1, 7, 64, 509, 3, 1024, 31, 256]
    offset = 0
    chunk_index = 0
    recovered_frames = 0
    while offset < len(transport):
        size = chunk_sizes[chunk_index % len(chunk_sizes)]
        chunk = transport[offset:offset + size]
        for record in record_parser.feed(chunk):
            recovered_frames += len(monitor.feed_record(record))
        offset += len(chunk)
        chunk_index += 1

    assert recovered_frames == 1100
    assert record_parser.bad_records == 0
    for port_id in range(11):
        assert monitor.statistics[port_id].sensor_frames == 100
        assert monitor.statistics[port_id].seq_lost == 0
        assert monitor.parser_errors(port_id) == 0

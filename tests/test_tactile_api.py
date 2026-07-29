import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from tactile_api import (
    CMD_ADC_DATA,
    CMD_COMBINED_DATA,
    SensorFrameEvent,
    StatusEvent,
    TactileCdcDecoder,
    decode_sensor_measurement,
)
from tactile_stream import (
    RECORD_DATA,
    RECORD_STATUS,
    SYSTEM_PORT_ID,
    SensorProtocolParser,
    build_sensor_frame,
    encode_record,
)


def sensor_frame(dev_id: int, command: int, data: bytes):
    parser = SensorProtocolParser()
    frames = parser.feed(build_sensor_frame(dev_id, command, data))
    assert len(frames) == 1
    return frames[0]


def test_adc_measurement_becomes_row_major_matrix_with_invalid_points():
    data = (
        (42).to_bytes(4, "little")
        + bytes([2, 3])
        + (9).to_bytes(2, "little")
        + (10).to_bytes(2, "little")
        + b"\xFF\xFF"
        + (12).to_bytes(2, "little")
        + (13).to_bytes(2, "little")
        + (14).to_bytes(2, "little")
        + (15).to_bytes(2, "little")
    )

    measurement = decode_sensor_measurement(0, sensor_frame(1, CMD_ADC_DATA, data))

    assert measurement is not None
    assert measurement.timestamp_ms == 42
    assert measurement.seq == 9
    assert measurement.adc_counts == ((10, None, 12), (13, 14, 15))
    assert measurement.normal_force_mn is None


def test_combined_measurement_decodes_signed_shear_and_invalid_values():
    data = (
        (100).to_bytes(4, "little")
        + bytes([1, 2])
        + (11).to_bytes(2, "little")
        + (1000).to_bytes(2, "little")
        + (-12).to_bytes(2, "little", signed=True)
        + (7).to_bytes(2, "little", signed=True)
        + b"\xFF\xFF" * 3
    )

    measurement = decode_sensor_measurement(4, sensor_frame(1, CMD_COMBINED_DATA, data))

    assert measurement is not None
    assert measurement.normal_force_mn == ((1000, None),)
    assert measurement.shear_x_mn == ((-12, None),)
    assert measurement.shear_y_mn == ((7, None),)


def test_stream_api_reassembles_one_combined_frame_across_records_and_chunks():
    rows = 12
    cols = 8
    data = bytearray((500).to_bytes(4, "little") + bytes([rows, cols]) + (33).to_bytes(2, "little"))
    for point in range(rows * cols):
        data.extend((point).to_bytes(2, "little"))
        data.extend((point - 50).to_bytes(2, "little", signed=True))
        data.extend((50 - point).to_bytes(2, "little", signed=True))
    frame = build_sensor_frame(1, CMD_COMBINED_DATA, bytes(data))
    transport = encode_record(RECORD_DATA, 10, frame[:512]) + encode_record(RECORD_DATA, 10, frame[512:])

    decoder = TactileCdcDecoder()
    events = []
    for offset in range(0, len(transport), 37):
        events.extend(decoder.feed(transport[offset:offset + 37]))

    decoded = [event for event in events if isinstance(event, SensorFrameEvent)]
    assert len(decoded) == 1
    measurement = decoded[0].measurement
    assert measurement is not None
    assert measurement.port_id == 10
    assert measurement.port_label == "CN11/UART12"
    assert measurement.rows == rows
    assert measurement.cols == cols
    assert measurement.normal_force_mn is not None
    assert measurement.normal_force_mn[11][7] == 95
    assert decoder.transport_bad_records == 0
    assert decoder.sensor_bad_frames(10) == 0


def test_stream_api_exposes_current_v4_status_records():
    payload = bytearray(236)
    payload[0] = 4
    payload[1] = 11
    payload[180:184] = (4186).to_bytes(4, "little")
    payload[228:232] = (1054).to_bytes(4, "little")

    decoder = TactileCdcDecoder()
    events = decoder.feed(encode_record(RECORD_STATUS, SYSTEM_PORT_ID, bytes(payload)))

    assert len(events) == 1
    assert isinstance(events[0], StatusEvent)
    assert events[0].status.usb_tx_high_water == 4186
    assert events[0].status.usb_tx_pending == 1054

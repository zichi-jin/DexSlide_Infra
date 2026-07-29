#!/usr/bin/env python3
"""Offline callability check for the tactile decoding API."""

from tactile_api import CMD_COMBINED_DATA, SensorFrameEvent, TactileCdcDecoder
from tactile_stream import RECORD_DATA, build_sensor_frame, encode_record


def build_combined_frame() -> bytes:
    rows = 12
    cols = 8
    data = bytearray((123456).to_bytes(4, "little") + bytes([rows, cols]) + (77).to_bytes(2, "little"))
    for point in range(rows * cols):
        if point == 3:
            data.extend(b"\xFF\xFF" * 3)
            continue
        data.extend((1000 + point).to_bytes(2, "little"))
        data.extend((point - 48).to_bytes(2, "little", signed=True))
        data.extend((48 - point).to_bytes(2, "little", signed=True))
    return build_sensor_frame(1, CMD_COMBINED_DATA, bytes(data))


def main() -> int:
    frame = build_combined_frame()
    assert len(frame) == 592
    transport = (
        encode_record(RECORD_DATA, 7, frame[:512])
        + encode_record(RECORD_DATA, 7, frame[512:])
    )

    decoder = TactileCdcDecoder()
    events = []
    for size in (1, 19, 64, 7, 513, 31, 1024):
        if not transport:
            break
        events.extend(decoder.feed(transport[:size]))
        transport = transport[size:]
    events.extend(decoder.feed(transport))

    measurement_events = [event for event in events if isinstance(event, SensorFrameEvent)]
    assert len(measurement_events) == 1
    measurement = measurement_events[0].measurement
    assert measurement is not None
    assert measurement.port_label == "CN8/UART9"
    assert measurement.seq == 77
    assert measurement.normal_force_mn is not None
    assert measurement.shear_x_mn is not None
    assert measurement.shear_y_mn is not None
    assert measurement.normal_force_mn[0][0] == 1000
    assert measurement.normal_force_mn[0][3] is None
    assert measurement.shear_x_mn[0][0] == -48
    assert measurement.shear_y_mn[0][0] == 48
    assert decoder.transport_bad_records == 0
    assert decoder.sensor_bad_frames(7) == 0
    print("tactile API smoke test passed: CN8 12x8 combined frame decoded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

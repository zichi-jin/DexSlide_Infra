#!/usr/bin/env python3
"""Public streaming API for STM32H562 tactile CDC data."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union

from tactile_stream import (
    PORT_LABELS,
    RECORD_DATA,
    RECORD_STATUS,
    SYSTEM_PORT_ID,
    SensorFrame,
    SensorProtocolParser,
    StreamStatus,
    TactileRecordParser,
    decode_status,
)


CMD_ADC_DATA = 0x20
CMD_FORCE_DATA = 0x21
CMD_COMBINED_DATA = 0x22
INVALID_U16 = 0xFFFF

Matrix = tuple[tuple[Optional[int], ...], ...]


@dataclass(frozen=True)
class TactileMeasurement:
    """One decoded active-report frame, with matrices indexed as [row][column]."""

    port_id: int
    port_label: str
    device_id: int
    command: int
    timestamp_ms: int
    seq: int
    rows: int
    cols: int
    raw_frame: bytes
    adc_counts: Optional[Matrix]
    normal_force_mn: Optional[Matrix]
    shear_x_mn: Optional[Matrix]
    shear_y_mn: Optional[Matrix]


@dataclass(frozen=True)
class SensorFrameEvent:
    """One sensor protocol frame recovered from an MCU data record."""

    port_id: int
    port_label: str
    record_flags: int
    frame: SensorFrame
    measurement: Optional[TactileMeasurement]


@dataclass(frozen=True)
class StatusEvent:
    """One MCU status record received from the CDC stream."""

    status: StreamStatus


TactileEvent = Union[SensorFrameEvent, StatusEvent]


def _matrix(values: list[Optional[int]], rows: int, cols: int) -> Matrix:
    return tuple(
        tuple(values[row * cols:(row + 1) * cols])
        for row in range(rows)
    )


def _decode_u16_or_none(data: bytes) -> Optional[int]:
    value = int.from_bytes(data, "little")
    return None if value == INVALID_U16 else value


def _decode_i16_or_none(data: bytes) -> Optional[int]:
    raw_value = int.from_bytes(data, "little")
    return None if raw_value == INVALID_U16 else int.from_bytes(data, "little", signed=True)


def decode_sensor_measurement(port_id: int, frame: SensorFrame) -> Optional[TactileMeasurement]:
    """Decode one validated sensor frame into a row-major two-dimensional measurement.

    Returns ``None`` for non-measurement commands. Raises ``ValueError`` when a
    measurement frame does not match its rows, columns, and command-specific size.
    """

    if port_id not in PORT_LABELS:
        raise ValueError(f"unsupported port id: {port_id}")
    if frame.cmd not in (CMD_ADC_DATA, CMD_FORCE_DATA, CMD_COMBINED_DATA):
        return None
    if len(frame.data) < 8:
        raise ValueError("measurement payload is shorter than its 8-byte header")

    timestamp_ms = int.from_bytes(frame.data[0:4], "little")
    rows = frame.data[4]
    cols = frame.data[5]
    seq = int.from_bytes(frame.data[6:8], "little")
    point_count = rows * cols
    bytes_per_point = 6 if frame.cmd == CMD_COMBINED_DATA else 2
    expected_length = 8 + point_count * bytes_per_point
    if rows == 0 or cols == 0 or len(frame.data) != expected_length:
        raise ValueError(
            f"invalid measurement dimensions or data length: "
            f"rows={rows}, cols={cols}, data={len(frame.data)}, expected={expected_length}"
        )

    body = frame.data[8:]
    common = {
        "port_id": port_id,
        "port_label": PORT_LABELS[port_id],
        "device_id": frame.dev_id,
        "command": frame.cmd,
        "timestamp_ms": timestamp_ms,
        "seq": seq,
        "rows": rows,
        "cols": cols,
        "raw_frame": frame.raw,
    }
    if frame.cmd == CMD_ADC_DATA:
        adc_values = [_decode_u16_or_none(body[offset:offset + 2]) for offset in range(0, len(body), 2)]
        return TactileMeasurement(
            **common,
            adc_counts=_matrix(adc_values, rows, cols),
            normal_force_mn=None,
            shear_x_mn=None,
            shear_y_mn=None,
        )
    if frame.cmd == CMD_FORCE_DATA:
        force_values = [_decode_u16_or_none(body[offset:offset + 2]) for offset in range(0, len(body), 2)]
        return TactileMeasurement(
            **common,
            adc_counts=None,
            normal_force_mn=_matrix(force_values, rows, cols),
            shear_x_mn=None,
            shear_y_mn=None,
        )

    normal_values = []
    shear_x_values = []
    shear_y_values = []
    for offset in range(0, len(body), 6):
        normal_values.append(_decode_u16_or_none(body[offset:offset + 2]))
        shear_x_values.append(_decode_i16_or_none(body[offset + 2:offset + 4]))
        shear_y_values.append(_decode_i16_or_none(body[offset + 4:offset + 6]))
    return TactileMeasurement(
        **common,
        adc_counts=None,
        normal_force_mn=_matrix(normal_values, rows, cols),
        shear_x_mn=_matrix(shear_x_values, rows, cols),
        shear_y_mn=_matrix(shear_y_values, rows, cols),
    )


class TactileCdcDecoder:
    """Stateful decoder for arbitrary CDC byte chunks from the tactile board."""

    def __init__(self) -> None:
        self._record_parser = TactileRecordParser()
        self._sensor_parsers = {port_id: SensorProtocolParser() for port_id in PORT_LABELS}

    @property
    def transport_bad_records(self) -> int:
        """Number of rejected MCU-PC records after CRC/format validation."""

        return self._record_parser.bad_records

    def sensor_bad_frames(self, port_id: int) -> int:
        """Number of rejected raw sensor frames for one connector."""

        if port_id not in self._sensor_parsers:
            raise ValueError(f"unsupported port id: {port_id}")
        return self._sensor_parsers[port_id].bad_frames

    def feed(self, chunk: bytes) -> list[TactileEvent]:
        """Decode every complete event available after appending one CDC chunk."""

        events: list[TactileEvent] = []
        for record in self._record_parser.feed(chunk):
            if record.record_type == RECORD_STATUS and record.port_id == SYSTEM_PORT_ID:
                events.append(StatusEvent(status=decode_status(record.payload)))
                continue
            if record.record_type != RECORD_DATA or record.port_id not in self._sensor_parsers:
                continue
            for frame in self._sensor_parsers[record.port_id].feed(record.payload):
                events.append(SensorFrameEvent(
                    port_id=record.port_id,
                    port_label=PORT_LABELS[record.port_id],
                    record_flags=record.flags,
                    frame=frame,
                    measurement=decode_sensor_measurement(record.port_id, frame),
                ))
        return events

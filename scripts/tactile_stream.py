#!/usr/bin/env python3
"""Read and validate multiplexed tactile-board CDC records."""

import argparse
import binascii
import glob
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional


SYNC = b"\xD3\x5C"
VERSION = 1
HEADER_SIZE = 8
CRC_SIZE = 2
RECORD_DATA = 0x01
RECORD_STATUS = 0x02
SYSTEM_PORT_ID = 0xFF
MAX_PAYLOAD_SIZE = 1024
STATUS_V1_PAYLOAD_SIZE = 96
STATUS_V2_PAYLOAD_SIZE = 184
STATUS_V3_PAYLOAD_SIZE = 228
STATUS_V4_PAYLOAD_SIZE = 236
SENSOR_HEAD = 0xA5
SENSOR_TAIL = 0x5A
MAX_SENSOR_PAYLOAD_SIZE = 4096

PORT_LABELS = {
    0: "CN1/USART1",
    1: "CN2/USART2",
    2: "CN3/USART3",
    3: "CN4/UART4",
    4: "CN5/UART5",
    5: "CN6/USART6",
    6: "CN7/UART7",
    7: "CN8/UART9",
    8: "CN9/USART10",
    9: "CN10/USART11",
    10: "CN11/UART12",
}


@dataclass(frozen=True)
class TactileRecord:
    record_type: int
    port_id: int
    flags: int
    payload: bytes


@dataclass(frozen=True)
class StreamStatus:
    version: int
    overflow: List[int]
    uart_errors: List[int]
    rx_halves: List[Optional[int]]
    last_uart_errors: List[Optional[int]]
    usb_tx_high_water: int
    usb_tx_pending: Optional[int]
    status_drops: Optional[int]
    last_dma_errors: List[Optional[int]]


@dataclass(frozen=True)
class SensorFrame:
    raw: bytes
    dev_id: int
    cmd: int
    data: bytes
    crc: int


@dataclass(frozen=True)
class SensorMeasurement:
    cmd: int
    rows: int
    cols: int
    seq: int
    timestamp_ms: int
    expected_data_length: int


def crc16_ccitt(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def _build_modbus_crc_table() -> tuple[int, ...]:
    table = []
    for value in range(256):
        crc = value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        table.append(crc)
    return tuple(table)


MODBUS_CRC_TABLE = _build_modbus_crc_table()


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ MODBUS_CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def build_sensor_frame(dev_id: int, cmd: int, data: bytes) -> bytes:
    body = bytes([dev_id, cmd]) + len(data).to_bytes(2, "little") + data
    return bytes([SENSOR_HEAD]) + body + crc16_modbus(body).to_bytes(2, "little") + bytes([SENSOR_TAIL])


def encode_record(record_type: int, port_id: int, payload: bytes, flags: int = 0) -> bytes:
    if len(payload) > 0xFFFF:
        raise ValueError("payload is too large")
    header = SYNC + bytes([VERSION, record_type, port_id, flags]) + len(payload).to_bytes(2, "little")
    crc = crc16_ccitt(header[2:] + payload)
    return header + payload + crc.to_bytes(2, "little")


class TactileRecordParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_records = 0

    def feed(self, chunk: bytes) -> List[TactileRecord]:
        self._buffer.extend(chunk)
        records: List[TactileRecord] = []

        while True:
            sync_index = self._buffer.find(SYNC)
            if sync_index < 0:
                if self._buffer:
                    self._buffer[:] = self._buffer[-1:] if self._buffer[-1] == SYNC[0] else b""
                break
            if sync_index > 0:
                del self._buffer[:sync_index]
            if len(self._buffer) < HEADER_SIZE:
                break

            version = self._buffer[2]
            record_type = self._buffer[3]
            port_id = self._buffer[4]
            flags = self._buffer[5]
            payload_length = int.from_bytes(self._buffer[6:8], "little")
            total_length = HEADER_SIZE + payload_length + CRC_SIZE

            if (version != VERSION or record_type not in (RECORD_DATA, RECORD_STATUS)
                    or payload_length > MAX_PAYLOAD_SIZE):
                self.bad_records += 1
                del self._buffer[0]
                continue
            if len(self._buffer) < total_length:
                break

            payload = bytes(self._buffer[HEADER_SIZE:HEADER_SIZE + payload_length])
            expected_crc = int.from_bytes(self._buffer[HEADER_SIZE + payload_length:total_length], "little")
            actual_crc = crc16_ccitt(bytes(self._buffer[2:HEADER_SIZE]) + payload)
            if expected_crc != actual_crc:
                self.bad_records += 1
                del self._buffer[0]
                continue

            records.append(TactileRecord(record_type, port_id, flags, payload))
            del self._buffer[:total_length]

        return records


class SensorProtocolParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_frames = 0

    def feed(self, chunk: bytes) -> List[SensorFrame]:
        self._buffer.extend(chunk)
        frames: List[SensorFrame] = []

        while True:
            head_index = self._buffer.find(bytes([SENSOR_HEAD]))
            if head_index < 0:
                self._buffer.clear()
                break
            if head_index > 0:
                del self._buffer[:head_index]
            if len(self._buffer) < 8:
                break

            data_length = int.from_bytes(self._buffer[3:5], "little")
            if data_length > MAX_SENSOR_PAYLOAD_SIZE:
                self.bad_frames += 1
                del self._buffer[0]
                continue

            frame_length = 8 + data_length
            if len(self._buffer) < frame_length:
                break

            candidate = bytes(self._buffer[:frame_length])
            expected_crc = int.from_bytes(candidate[5 + data_length:7 + data_length], "little")
            if candidate[-1] != SENSOR_TAIL or expected_crc != crc16_modbus(candidate[1:5 + data_length]):
                self.bad_frames += 1
                del self._buffer[0]
                continue

            frames.append(SensorFrame(
                raw=candidate,
                dev_id=candidate[1],
                cmd=candidate[2],
                data=candidate[5:5 + data_length],
                crc=expected_crc,
            ))
            del self._buffer[:frame_length]

        return frames


def decode_measurement(frame: SensorFrame) -> Optional[SensorMeasurement]:
    if frame.cmd not in (0x20, 0x21, 0x22) or len(frame.data) < 8:
        return None

    rows = frame.data[4]
    cols = frame.data[5]
    point_count = rows * cols
    bytes_per_point = 6 if frame.cmd == 0x22 else 2
    return SensorMeasurement(
        cmd=frame.cmd,
        rows=rows,
        cols=cols,
        seq=int.from_bytes(frame.data[6:8], "little"),
        timestamp_ms=int.from_bytes(frame.data[0:4], "little"),
        expected_data_length=8 + point_count * bytes_per_point,
    )


def decode_status(payload: bytes) -> StreamStatus:
    version = payload[0] if payload else 0
    expected_length = {
        1: STATUS_V1_PAYLOAD_SIZE,
        2: STATUS_V2_PAYLOAD_SIZE,
        3: STATUS_V3_PAYLOAD_SIZE,
        4: STATUS_V4_PAYLOAD_SIZE,
    }.get(version, 0)
    if len(payload) != expected_length:
        raise ValueError(f"unexpected status payload size: {len(payload)}")
    if payload[1] != len(PORT_LABELS):
        raise ValueError("unsupported status payload")
    overflow = [int.from_bytes(payload[4 + 4 * i:8 + 4 * i], "little") for i in PORT_LABELS]
    uart_errors = [int.from_bytes(payload[48 + 4 * i:52 + 4 * i], "little") for i in PORT_LABELS]
    if version >= 2:
        rx_halves = [int.from_bytes(payload[92 + 4 * i:96 + 4 * i], "little") for i in PORT_LABELS]
        last_uart_errors = [int.from_bytes(payload[136 + 4 * i:140 + 4 * i], "little") for i in PORT_LABELS]
        usb_tx_high_water = int.from_bytes(payload[180:184], "little")
        last_dma_errors = (
            [int.from_bytes(payload[184 + 4 * i:188 + 4 * i], "little") for i in PORT_LABELS]
            if version >= 3 else [None] * len(PORT_LABELS)
        )
        usb_tx_pending = int.from_bytes(payload[228:232], "little") if version >= 4 else None
        status_drops = int.from_bytes(payload[232:236], "little") if version >= 4 else None
    else:
        rx_halves = [None] * len(PORT_LABELS)
        last_uart_errors = [None] * len(PORT_LABELS)
        usb_tx_high_water = int.from_bytes(payload[92:96], "little")
        usb_tx_pending = None
        status_drops = None
        last_dma_errors = [None] * len(PORT_LABELS)
    return StreamStatus(
        version=version,
        overflow=overflow,
        uart_errors=uart_errors,
        rx_halves=rx_halves,
        last_uart_errors=last_uart_errors,
        usb_tx_high_water=usb_tx_high_water,
        usb_tx_pending=usb_tx_pending,
        status_drops=status_drops,
        last_dma_errors=last_dma_errors,
    )


def format_uart_error(error_code: int) -> str:
    labels = ((0x01, "PE"), (0x02, "NE"), (0x04, "FE"), (0x08, "ORE"), (0x10, "DMA"), (0x20, "RTO"))
    names = [name for mask, name in labels if error_code & mask]
    return "|".join(names) if names else "none"


def format_dma_error(error_code: int) -> str:
    labels = ((0x01, "DTE"), (0x02, "ULE"), (0x04, "USE"), (0x08, "TO"),
              (0x10, "timeout"), (0x20, "no_xfer"), (0x40, "busy"))
    names = [name for mask, name in labels if error_code & mask]
    return "|".join(names) if names else "none"


def format_status(payload: bytes) -> str:
    status = decode_status(payload)
    entries = []
    for port_id, label in PORT_LABELS.items():
        overflow = status.overflow[port_id]
        errors = status.uart_errors[port_id]
        if overflow or errors:
            entry = f"{label}: overflow={overflow}, uart_error={errors}"
            last_error = status.last_uart_errors[port_id]
            if last_error is not None:
                entry += f", last_error=0x{last_error:02X}({format_uart_error(last_error)})"
            dma_error = status.last_dma_errors[port_id]
            if dma_error:
                entry += f", dma_error=0x{dma_error:02X}({format_dma_error(dma_error)})"
            entries.append(entry)
    active_ports = sum(count is not None and count > 0 for count in status.rx_halves)
    prefix = f"status v{status.version} usb_tx_high_water={status.usb_tx_high_water} B"
    if status.usb_tx_pending is not None:
        prefix += f", usb_tx_pending={status.usb_tx_pending} B"
    if status.status_drops is not None:
        prefix += f", status_drop={status.status_drops}"
    prefix += f", rx_active={active_ports}/{len(PORT_LABELS)}"
    return prefix if not entries else prefix + "; " + "; ".join(entries)


@dataclass
class PortStatistics:
    raw_bytes: int = 0
    records: int = 0
    sensor_frames: int = 0
    sensor_length_errors: int = 0
    seq_lost: int = 0
    seq_resets: int = 0
    last_seq: Optional[int] = None


class TactileStreamMonitor:
    def __init__(self) -> None:
        self.sensor_parsers = {port_id: SensorProtocolParser() for port_id in PORT_LABELS}
        self.statistics = {port_id: PortStatistics() for port_id in PORT_LABELS}

    def feed_record(self, record: TactileRecord) -> List[SensorFrame]:
        if record.record_type != RECORD_DATA or record.port_id not in self.statistics:
            return []

        stats = self.statistics[record.port_id]
        stats.raw_bytes += len(record.payload)
        stats.records += 1
        frames = self.sensor_parsers[record.port_id].feed(record.payload)
        for frame in frames:
            stats.sensor_frames += 1
            measurement = decode_measurement(frame)
            if measurement is None:
                continue
            if len(frame.data) != measurement.expected_data_length:
                stats.sensor_length_errors += 1
                continue
            if stats.last_seq is not None:
                delta = (measurement.seq - stats.last_seq) & 0xFFFF
                if delta == 0:
                    continue
                if delta <= 0x8000:
                    stats.seq_lost += delta - 1
                else:
                    stats.seq_resets += 1
            stats.last_seq = measurement.seq
        return frames

    def parser_errors(self, port_id: int) -> int:
        return self.sensor_parsers[port_id].bad_frames


def choose_default_port() -> str:
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else ""


def format_frame(port_id: int, frame: SensorFrame) -> str:
    measurement = decode_measurement(frame)
    if measurement is None:
        return f"{PORT_LABELS[port_id]} cmd=0x{frame.cmd:02X} data={len(frame.data)} B"
    return (
        f"{PORT_LABELS[port_id]} cmd=0x{measurement.cmd:02X} rows={measurement.rows} cols={measurement.cols} "
        f"seq={measurement.seq} ts={measurement.timestamp_ms} ms data={len(frame.data)} B"
    )


def format_statistics(monitor: TactileStreamMonitor, interval_s: float, record_bad: int) -> str:
    entries = []
    for port_id, label in PORT_LABELS.items():
        stats = monitor.statistics[port_id]
        if stats.records == 0:
            continue
        rate_kib_s = stats.raw_bytes / max(interval_s, 0.001) / 1024
        entries.append(
            f"{label}: records={stats.records}, raw={stats.raw_bytes} B, frames={stats.sensor_frames}, "
            f"rate={rate_kib_s:.1f} KiB/s, seq_lost={stats.seq_lost}, "
            f"frame_bad={monitor.parser_errors(port_id) + stats.sensor_length_errors}"
        )
    summary = " | ".join(entries)
    return f"record_bad={record_bad}" if not summary else f"record_bad={record_bad} | {summary}"


def read_stream(port: str, baudrate: int, show_hex: bool, show_frames: bool) -> int:
    try:
        import serial  # type: ignore
    except ModuleNotFoundError:
        print("需要 pyserial：python3 -m pip install pyserial", file=sys.stderr)
        return 2

    parser = TactileRecordParser()
    monitor = TactileStreamMonitor()
    last_report = time.monotonic()

    with serial.Serial(port, baudrate, timeout=0.1) as ser:
        ser.dtr = True
        print(f"listening on {port} at {baudrate} baud")
        try:
            while True:
                chunk = ser.read(4096)
                for record in parser.feed(chunk):
                    if record.record_type == RECORD_DATA:
                        if record.port_id not in PORT_LABELS:
                            print(f"unknown data port: {record.port_id}", file=sys.stderr)
                            continue
                        frames = monitor.feed_record(record)
                        if show_hex:
                            print(f"{PORT_LABELS[record.port_id]} {record.payload.hex(' ')}")
                        if show_frames:
                            for frame in frames:
                                print(format_frame(record.port_id, frame))
                    elif record.record_type == RECORD_STATUS and record.port_id == SYSTEM_PORT_ID:
                        print(format_status(record.payload))

                now = time.monotonic()
                if now - last_report >= 1.0:
                    report = format_statistics(monitor, now - last_report, parser.bad_records)
                    if report:
                        print(report)
                    for stats in monitor.statistics.values():
                        stats.raw_bytes = 0
                        stats.records = 0
                        stats.sensor_frames = 0
                        stats.sensor_length_errors = 0
                    last_report = now
        except KeyboardInterrupt:
            print()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Read STM32H562 tactile stream records")
    parser.add_argument("--port", default=choose_default_port(), help="CDC device, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="CDC line-setting value")
    parser.add_argument("--hex", action="store_true", help="print every raw DMA payload")
    parser.add_argument("--frames", action="store_true", help="print every CRC-valid sensor frame")
    args = parser.parse_args()
    if not args.port:
        parser.error("未找到串口，请用 --port 指定 /dev/ttyACM*")
    return read_stream(args.port, args.baud, args.hex, args.frames)


if __name__ == "__main__":
    raise SystemExit(main())

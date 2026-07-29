#!/usr/bin/env python3
import argparse
import cmd
import json
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ModuleNotFoundError:
    serial = None

    class _PortInfo:
        def __init__(self, device: str) -> None:
            self.device = device

    class _ListPorts:
        @staticmethod
        def comports() -> List[_PortInfo]:
            import glob

            devices = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
            return [_PortInfo(device) for device in devices]

    list_ports = _ListPorts()  # type: ignore

import fcntl
import glob
import os
import termios


BRIDGE_SYNC = b"\xA5\x5A"
BRIDGE_HEADER_SIZE = 5
SENSOR_HEAD = 0xA5
SENSOR_TAIL = 0x5A
DEFAULT_BAUD = 115200
DEFAULT_DEV_ID = 0x01
MAX_BRIDGE_FRAME = 4096
DEFAULT_SCAN_TIMEOUT = 0.4
LIVE_HEX_REFRESH_S = 0.2
RAW_STREAM_WINDOW_SIZE = 512
LIVE_HEX_RAW_TAIL_BYTES = 24

PORT_LABELS = {
    0: "UART4",
    1: "UART5",
    2: "UART7",
    3: "UART9",
    4: "UART12",
    5: "USART1",
    6: "USART2",
    7: "USART3",
    8: "USART6",
    9: "USART10",
    10: "USART11",
}
PORT_NAME_TO_ID = {label.lower(): port_id for port_id, label in PORT_LABELS.items()}
CMD_NAMES = {
    0x01: "DISCOVER",
    0x02: "DEVICE_INFO",
    0x10: "CONFIG_MODE",
    0x11: "SET_MODE",
    0x12: "SET_SHEAR",
    0x13: "SET_ALGO",
    0x14: "SET_FREQ",
    0x20: "ADC_DATA",
    0x21: "FORCE_DATA",
    0x22: "COMBINED_DATA",
    0x30: "MODEL_LOAD",
    0x35: "NN_MODEL_LOAD",
    0x36: "NN_MODEL_START",
    0x37: "NN_MODEL_CHUNK",
    0x38: "NN_MODEL_FINISH",
    0x50: "REBOOT",
}
COMMAND_NAME_TO_CODE = {
    name.upper(): code for code, name in CMD_NAMES.items()
}
COMMAND_NAME_TO_CODE.update({
    f"CMD_{name.upper()}": code for code, name in CMD_NAMES.items()
})
MODE_NAMES = {0: "ADC", 1: "FORCE", 2: "COMBINED"}
ALGO_NAMES = {0: "POLY", 1: "CNN"}
MONITOR_MODES = {"summary", "raw", "quiet"}


@dataclass
class SensorFrame:
    raw: bytes
    dev_id: int
    cmd: int
    data: bytes
    crc: int


class BridgeStreamParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.text_buffer = bytearray()

    def _consume_text(self, chunk: bytes) -> List[str]:
        if not chunk:
            return []
        self.text_buffer.extend(chunk)
        lines: List[str] = []
        while True:
            idx = self.text_buffer.find(b"\n")
            if idx < 0:
                break
            line = self.text_buffer[:idx].decode(errors="replace").rstrip("\r")
            del self.text_buffer[: idx + 1]
            if line:
                lines.append(line)
        return lines

    def feed(self, data: bytes) -> Tuple[List[str], List[Tuple[int, bytes]]]:
        self.buffer.extend(data)
        text_lines: List[str] = []
        frames: List[Tuple[int, bytes]] = []

        while True:
            idx = self.buffer.find(BRIDGE_SYNC)
            if idx < 0:
                if self.buffer:
                    keep = 1 if self.buffer[-1] == BRIDGE_SYNC[0] else 0
                    chunk = bytes(self.buffer[:-keep] if keep else self.buffer)
                    text_lines.extend(self._consume_text(chunk))
                    if keep:
                        self.buffer[:] = self.buffer[-1:]
                    else:
                        self.buffer.clear()
                break

            if idx > 0:
                text_lines.extend(self._consume_text(bytes(self.buffer[:idx])))
                del self.buffer[:idx]

            if len(self.buffer) < BRIDGE_HEADER_SIZE:
                break

            payload_len = self.buffer[3] | (self.buffer[4] << 8)
            if payload_len > MAX_BRIDGE_FRAME:
                text_lines.extend(self._consume_text(bytes(self.buffer[:1])))
                del self.buffer[:1]
                continue

            frame_len = BRIDGE_HEADER_SIZE + payload_len
            if len(self.buffer) < frame_len:
                break

            port_id = self.buffer[2]
            payload = bytes(self.buffer[BRIDGE_HEADER_SIZE:frame_len])
            frames.append((port_id, payload))
            del self.buffer[:frame_len]

        return text_lines, frames


class LinuxSerial:
    def __init__(self, port: str, baudrate: int, timeout: float = 0.05) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._configure()

    def _baud_constant(self) -> int:
        name = f"B{self.baudrate}"
        return getattr(termios, name, termios.B115200)

    def _configure(self) -> None:
        attrs = termios.tcgetattr(self.fd)
        iflag = 0
        oflag = 0
        cflag = attrs[2] | termios.CLOCAL | termios.CREAD
        lflag = 0
        cc = attrs[6]

        attrs[0] = iflag
        attrs[1] = oflag
        attrs[2] = cflag
        attrs[3] = lflag
        attrs[4] = self._baud_constant()
        attrs[5] = self._baud_constant()
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = max(1, int(self.timeout * 10))

        if hasattr(termios, "cfmakeraw"):
            termios.cfmakeraw(attrs)
            attrs[2] |= termios.CLOCAL | termios.CREAD
            attrs[4] = self._baud_constant()
            attrs[5] = self._baud_constant()
            cc[termios.VMIN] = 0
            cc[termios.VTIME] = max(1, int(self.timeout * 10))

        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        try:
            fcntl.fcntl(self.fd, fcntl.F_SETFL, os.O_NONBLOCK)
        except OSError:
            pass

    def read(self, size: int) -> bytes:
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b""
        except OSError:
            return b""

    def write(self, data: bytes) -> int:
        return os.write(self.fd, data)

    def flush(self) -> None:
        termios.tcdrain(self.fd)

    def reset_input_buffer(self) -> None:
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def __enter__(self) -> "LinuxSerial":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def open_serial(port: str, baud: int, timeout: float):
    if serial is not None:
        return serial.Serial(port, baud, timeout=timeout)
    return LinuxSerial(port, baud, timeout=timeout)


class SensorProtocolParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, chunk: bytes) -> List[SensorFrame]:
        self.buffer.extend(chunk)
        frames: List[SensorFrame] = []

        while True:
            head_idx = self.buffer.find(bytes([SENSOR_HEAD]))
            if head_idx < 0:
                self.buffer.clear()
                break
            if head_idx > 0:
                del self.buffer[:head_idx]
            if len(self.buffer) < 8:
                break

            payload_len = self.buffer[3] | (self.buffer[4] << 8)
            frame_len = 8 + payload_len
            if frame_len > MAX_BRIDGE_FRAME:
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_len:
                break

            candidate = bytes(self.buffer[:frame_len])
            if candidate[-1] != SENSOR_TAIL:
                del self.buffer[0]
                continue

            crc_expected = candidate[5 + payload_len] | (candidate[6 + payload_len] << 8)
            crc_actual = crc16_modbus(candidate[1:5 + payload_len])
            if crc_expected != crc_actual:
                del self.buffer[0]
                continue

            frames.append(
                SensorFrame(
                    raw=candidate,
                    dev_id=candidate[1],
                    cmd=candidate[2],
                    data=candidate[5:5 + payload_len],
                    crc=crc_expected,
                )
            )
            del self.buffer[:frame_len]

        return frames


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_sensor_frame(dev_id: int, cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([dev_id & 0xFF, cmd & 0xFF, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    crc = crc16_modbus(body)
    return bytes([SENSOR_HEAD]) + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF, SENSOR_TAIL])


def build_bridge_frame(port_id: int, payload: bytes) -> bytes:
    if not 0 <= port_id <= 0xFF:
        raise ValueError(f"invalid port_id: {port_id}")
    if len(payload) > 0xFFFF:
        raise ValueError("payload too large for bridge frame")
    return BRIDGE_SYNC + bytes([port_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload


def decode_ascii(raw: bytes) -> str:
    return raw.decode(errors="replace").rstrip("\x00")


def parse_device_info(data: bytes) -> Dict[str, object]:
    return {
        "version": decode_ascii(data[0:8]).strip(),
        "sn": decode_ascii(data[8:24]).strip(),
        "rows": data[24] if len(data) > 24 else None,
        "cols": data[25] if len(data) > 25 else None,
        "freq": int.from_bytes(data[30:32], "little") if len(data) >= 32 else None,
        "out_mode": data[32] if len(data) > 32 else None,
        "algo": data[33] if len(data) > 33 else None,
        "shear_enabled": data[34] if len(data) > 34 else None,
    }


def reshape_matrix(values: Sequence[int], rows: int, cols: int) -> List[List[int]]:
    return [list(values[r * cols:(r + 1) * cols]) for r in range(rows)]


def decode_sensor_frame(frame: SensorFrame) -> Dict[str, object]:
    result: Dict[str, object] = {
        "dev_id": frame.dev_id,
        "cmd": frame.cmd,
        "cmd_name": CMD_NAMES.get(frame.cmd, f"0x{frame.cmd:02X}"),
        "data_len": len(frame.data),
        "raw_hex": frame.raw.hex(" "),
    }

    if frame.cmd == 0x02 and len(frame.data) >= 40:
        result["device_info"] = parse_device_info(frame.data)
        return result

    if frame.cmd in (0x20, 0x21, 0x22) and len(frame.data) >= 8:
        rows = frame.data[4]
        cols = frame.data[5]
        seq = int.from_bytes(frame.data[6:8], "little")
        timestamp = int.from_bytes(frame.data[0:4], "little")
        body = frame.data[8:]
        n = rows * cols
        result.update({
            "timestamp_ms": timestamp,
            "rows": rows,
            "cols": cols,
            "seq": seq,
        })

        if frame.cmd in (0x20, 0x21):
            expected = 2 * n
            if len(body) == expected:
                values = [int.from_bytes(body[2 * i:2 * i + 2], "little", signed=False) for i in range(n)]
                result["matrix"] = reshape_matrix(values, rows, cols)
            else:
                result["decode_error"] = f"payload body len {len(body)} != expected {expected}"
        else:
            expected = 6 * n
            if len(body) == expected:
                fn = []
                fx = []
                fy = []
                for i in range(n):
                    off = 6 * i
                    fn.append(int.from_bytes(body[off:off + 2], "little", signed=False))
                    fx.append(int.from_bytes(body[off + 2:off + 4], "little", signed=True))
                    fy.append(int.from_bytes(body[off + 4:off + 6], "little", signed=True))
                result["Fn"] = reshape_matrix(fn, rows, cols)
                result["Fx"] = reshape_matrix(fx, rows, cols)
                result["Fy"] = reshape_matrix(fy, rows, cols)
            else:
                result["decode_error"] = f"payload body len {len(body)} != expected {expected}"
        return result

    if len(frame.data) == 1 and frame.cmd in (0x10, 0x11, 0x12, 0x13, 0x14):
        result["ack"] = "success" if frame.data[0] == 0x00 else "failure"
        result["status"] = frame.data[0]
        return result

    result["data_hex"] = frame.data.hex(" ")
    return result


def summarize_decoded_frame(decoded: Dict[str, object]) -> str:
    cmd_name = decoded["cmd_name"]
    if "device_info" in decoded:
        info = decoded["device_info"]
        out_mode = info.get("out_mode")
        out_mode_name = MODE_NAMES.get(out_mode, out_mode)
        algo = info.get("algo")
        algo_name = ALGO_NAMES.get(algo, algo)
        return (
            f"{cmd_name} version={info.get('version')} sn={info.get('sn')} "
            f"rows={info.get('rows')} cols={info.get('cols')} freq={info.get('freq')}Hz "
            f"mode={out_mode_name} algo={algo_name} shear={info.get('shear_enabled')}"
        )
    if decoded.get("cmd") in (0x20, 0x21, 0x22):
        return (
            f"{cmd_name} rows={decoded.get('rows')} cols={decoded.get('cols')} "
            f"seq={decoded.get('seq')} ts={decoded.get('timestamp_ms')}ms"
        )
    if "ack" in decoded:
        return f"{cmd_name} ack={decoded['ack']}"
    if "data_hex" in decoded:
        return f"{cmd_name} data={decoded['data_hex']}"
    return cmd_name


def port_label(port_id: int) -> str:
    return PORT_LABELS.get(port_id, f"PORT{port_id}")


def parse_port_list(spec: str) -> List[int]:
    spec = spec.strip()
    if not spec:
        raise ValueError("empty port spec")
    if spec.lower() == "all":
        return list(PORT_LABELS)
    if spec.lower() == "none":
        return []

    ports: List[int] = []
    for part in spec.replace(",", " ").split():
        key = part.lower()
        if key in PORT_NAME_TO_ID:
            ports.append(PORT_NAME_TO_ID[key])
            continue
        port_id = int(part, 0)
        if port_id not in PORT_LABELS:
            raise ValueError(f"unknown port: {part}")
        ports.append(port_id)
    return sorted(dict.fromkeys(ports))


def parse_hex_payload(text: str) -> bytes:
    cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    tokens = [token for token in cleaned.split() if token]
    if not tokens:
        return b""
    if len(tokens) == 1 and len(tokens[0]) % 2 == 0:
        return bytes.fromhex(tokens[0])
    return bytes(int(token, 16) & 0xFF for token in tokens)


def command_code_from_name(token: str) -> Optional[int]:
    return COMMAND_NAME_TO_CODE.get(token.strip().upper())


def extract_latest_a5_5a_frame(stream: bytes) -> bytes:
    latest = b""
    size = len(stream)
    i = 0

    while i < size:
        if stream[i] != SENSOR_HEAD:
            i += 1
            continue

        if i + 8 > size:
            break

        payload_len = stream[i + 3] | (stream[i + 4] << 8)
        frame_len = 8 + payload_len
        end = i + frame_len
        if end > size:
            break

        candidate = stream[i:end]
        if candidate[-1] == SENSOR_TAIL:
            latest = candidate
            i = end
            continue

        i += 1

    return latest


def encode_named_command_payload(cmd_code: int, args: Sequence[str]) -> bytes:
    if cmd_code in (0x01, 0x02, 0x50):
        if args:
            raise ValueError(f"{CMD_NAMES[cmd_code]} does not take a payload")
        return b""

    if cmd_code == 0x10:
        if len(args) != 1:
            raise ValueError("CMD_CONFIG_MODE expects enter|exit|stop|resume|0|1")
        mapping = {"exit": 0, "resume": 0, "0": 0, "enter": 1, "stop": 1, "1": 1}
        key = args[0].strip().lower()
        if key not in mapping:
            raise ValueError("CMD_CONFIG_MODE expects enter|exit|stop|resume|0|1")
        return bytes([mapping[key]])

    if cmd_code == 0x11:
        if len(args) != 1:
            raise ValueError("CMD_SET_MODE expects adc|force|combined|0|1|2")
        mapping = {"adc": 0, "force": 1, "combined": 2, "0": 0, "1": 1, "2": 2}
        key = args[0].strip().lower()
        if key not in mapping:
            raise ValueError("CMD_SET_MODE expects adc|force|combined|0|1|2")
        return bytes([mapping[key]])

    if cmd_code == 0x13:
        if len(args) != 1:
            raise ValueError("CMD_SET_ALGO expects poly|cnn|0|1")
        mapping = {"poly": 0, "cnn": 1, "0": 0, "1": 1}
        key = args[0].strip().lower()
        if key not in mapping:
            raise ValueError("CMD_SET_ALGO expects poly|cnn|0|1")
        return bytes([mapping[key]])

    if cmd_code == 0x14:
        if len(args) != 1:
            raise ValueError("CMD_SET_FREQ expects one integer Hz value")
        hz = int(args[0], 0)
        if not 50 <= hz <= 100:
            raise ValueError("CMD_SET_FREQ expects 50..100 Hz")
        return hz.to_bytes(2, "little", signed=False)

    if not args:
        return b""

    return parse_hex_payload(" ".join(args))


def pick_default_port() -> str:
    ports = [p.device for p in list_ports.comports()]
    if not ports:
        return ""
    for candidate in ports:
        if "ttyACM" in candidate or "ttyUSB" in candidate:
            return candidate
    return ports[0]


class TactileSession:
    def __init__(self, ser, dev_id: int, target_ports: List[int], monitor: str) -> None:
        self.ser = ser
        self.default_dev_id = dev_id
        self.target_ports = target_ports
        self.watch_ports = set(PORT_LABELS)
        self.monitor = monitor
        self.latest_frames: Dict[int, SensorFrame] = {}
        self.latest_decoded_frames: Dict[int, Dict[str, object]] = {}
        self.bridge_activity_counts: Dict[int, int] = {}
        self.bridge_last_seen_times: Dict[int, float] = {}
        self.bridge_latest_payloads: Dict[int, bytes] = {}
        self.bridge_latest_sensor_windows: Dict[int, bytes] = {}
        self.bridge_recent_streams: Dict[int, bytes] = {}
        self.bridge_total_bytes: Dict[int, int] = {}
        self.device_info_counts: Dict[int, int] = {}
        self.last_device_info_frames: Dict[int, Dict[str, object]] = {}
        self.last_scan_active_ports: List[int] = []
        self.bridge_parser = BridgeStreamParser()
        self.sensor_parsers = {port_id: SensorProtocolParser() for port_id in PORT_LABELS}
        self.stop_event = threading.Event()
        self.print_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.reader = threading.Thread(target=self._reader_loop, name="tactile-cli-reader", daemon=True)

    def start(self) -> None:
        self.reader.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.reader.join(timeout=1.0)

    def _async_print(self, text: str) -> None:
        with self.print_lock:
            sys.stdout.write(f"\n{text}\ncli> ")
            sys.stdout.flush()

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            chunk = self.ser.read(512)
            if not chunk:
                continue
            text_lines, bridge_frames = self.bridge_parser.feed(chunk)
            for line in text_lines:
                self._async_print(f"[bridge] {line}")
            for port_id, payload in bridge_frames:
                self._handle_bridge_frame(port_id, payload)

    def _handle_bridge_frame(self, port_id: int, payload: bytes) -> None:
        now = time.monotonic()
        with self.state_lock:
            self.bridge_activity_counts[port_id] = self.bridge_activity_counts.get(port_id, 0) + 1
            self.bridge_last_seen_times[port_id] = now
            self.bridge_latest_payloads[port_id] = bytes(payload)
            recent_stream = self.bridge_recent_streams.get(port_id, b"") + bytes(payload)
            if len(recent_stream) > RAW_STREAM_WINDOW_SIZE:
                recent_stream = recent_stream[-RAW_STREAM_WINDOW_SIZE:]
            self.bridge_recent_streams[port_id] = recent_stream
            self.bridge_latest_sensor_windows[port_id] = extract_latest_a5_5a_frame(recent_stream)
            self.bridge_total_bytes[port_id] = self.bridge_total_bytes.get(port_id, 0) + len(payload)

        if self.monitor == "raw" and port_id in self.watch_ports:
            self._async_print(f"[{port_label(port_id)}] bridge {payload.hex(' ')}")

        parser = self.sensor_parsers.get(port_id)
        if parser is None:
            return

        for frame in parser.feed(payload):
            decoded = decode_sensor_frame(frame)
            now = time.monotonic()
            with self.state_lock:
                self.latest_frames[port_id] = frame
                self.latest_decoded_frames[port_id] = decoded
                if decoded.get("cmd") == 0x02 and "device_info" in decoded:
                    self.device_info_counts[port_id] = self.device_info_counts.get(port_id, 0) + 1
                    self.last_device_info_frames[port_id] = dict(decoded)
            if port_id not in self.watch_ports or self.monitor == "quiet":
                continue
            if self.monitor == "summary":
                self._async_print(f"[{port_label(port_id)}] {summarize_decoded_frame(decoded)}")
            elif self.monitor == "raw":
                self._async_print(f"[{port_label(port_id)}] sensor {frame.raw.hex(' ')}")

    def send_bridge_payload(self, port_id: int, payload: bytes) -> None:
        self.ser.write(build_bridge_frame(port_id, payload))
        self.ser.flush()

    def send_sensor_frame(self, sensor_frame: bytes, ports: Optional[Iterable[int]] = None) -> None:
        target_ports = list(self.target_ports if ports is None else ports)
        if not target_ports:
            raise ValueError("no target ports selected")
        for port_id in target_ports:
            self.send_bridge_payload(port_id, sensor_frame)

    def send_protocol_command(
        self,
        cmd_code: int,
        payload: bytes = b"",
        dev_id: Optional[int] = None,
        ports: Optional[Iterable[int]] = None,
    ) -> None:
        actual_dev_id = self.default_dev_id if dev_id is None else dev_id
        frame = build_sensor_frame(actual_dev_id, cmd_code, payload)
        self.send_sensor_frame(frame, ports=ports)

    def describe_ports(self) -> str:
        lines = [f"{port_id}: {label}" for port_id, label in PORT_LABELS.items()]
        lines.append(f"target={','.join(port_label(p) for p in self.target_ports) or '<none>'}")
        lines.append(f"watch={','.join(port_label(p) for p in sorted(self.watch_ports)) or '<none>'}")
        lines.append(f"monitor={self.monitor}")
        lines.append(f"default_dev_id=0x{self.default_dev_id:02X}")
        return "\n".join(lines)

    def snapshot_frames(self) -> Dict[int, SensorFrame]:
        with self.state_lock:
            return dict(self.latest_frames)

    def snapshot_decoded_frames(self) -> Dict[int, Dict[str, object]]:
        with self.state_lock:
            return dict(self.latest_decoded_frames)

    def snapshot_bridge_activity_counts(self) -> Dict[int, int]:
        with self.state_lock:
            return dict(self.bridge_activity_counts)

    def snapshot_bridge_last_seen_times(self) -> Dict[int, float]:
        with self.state_lock:
            return dict(self.bridge_last_seen_times)

    def snapshot_bridge_latest_payloads(self) -> Dict[int, bytes]:
        with self.state_lock:
            return dict(self.bridge_latest_payloads)

    def snapshot_bridge_latest_sensor_windows(self) -> Dict[int, bytes]:
        with self.state_lock:
            return dict(self.bridge_latest_sensor_windows)

    def snapshot_bridge_recent_streams(self) -> Dict[int, bytes]:
        with self.state_lock:
            return dict(self.bridge_recent_streams)

    def snapshot_bridge_total_bytes(self) -> Dict[int, int]:
        with self.state_lock:
            return dict(self.bridge_total_bytes)

    def snapshot_device_info_counts(self) -> Dict[int, int]:
        with self.state_lock:
            return dict(self.device_info_counts)

    def snapshot_device_info_frames(self) -> Dict[int, Dict[str, object]]:
        with self.state_lock:
            return dict(self.last_device_info_frames)

    def observe_ports(
        self,
        ports: Sequence[int],
        timeout_s: float = DEFAULT_SCAN_TIMEOUT,
    ) -> Dict[int, Dict[str, object]]:
        if not ports:
            raise ValueError("no ports selected for scan")

        before_counts = self.snapshot_bridge_activity_counts()

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            current_counts = self.snapshot_bridge_activity_counts()
            if all(current_counts.get(port_id, 0) > before_counts.get(port_id, 0) for port_id in ports):
                break
            time.sleep(0.01)

        current_counts = self.snapshot_bridge_activity_counts()
        current_last_times = self.snapshot_bridge_last_seen_times()
        now = time.monotonic()
        results: Dict[int, Dict[str, object]] = {}
        active_ports: List[int] = []
        for port_id in ports:
            frame_count_delta = current_counts.get(port_id, 0) - before_counts.get(port_id, 0)
            last_seen_time = current_last_times.get(port_id)
            last_seen_ms = None if last_seen_time is None else int((now - last_seen_time) * 1000.0)
            if frame_count_delta > 0:
                active_ports.append(port_id)
            results[port_id] = {
                "has_activity": frame_count_delta > 0,
                "activity_count": frame_count_delta,
                "last_seen_ms": last_seen_ms,
            }
        with self.state_lock:
            self.last_scan_active_ports = active_ports
        return results

    def query_device_info(self, ports: Sequence[int], timeout_s: float = DEFAULT_SCAN_TIMEOUT) -> Dict[int, Optional[Dict[str, object]]]:
        if not ports:
            raise ValueError("no ports selected for query")

        before_info_counts = self.snapshot_device_info_counts()
        self.send_protocol_command(0x02, ports=ports)

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            current_info_counts = self.snapshot_device_info_counts()
            if all(current_info_counts.get(port_id, 0) > before_info_counts.get(port_id, 0) for port_id in ports):
                break
            time.sleep(0.01)

        current_info_counts = self.snapshot_device_info_counts()
        current_info_frames = self.snapshot_device_info_frames()
        results: Dict[int, Optional[Dict[str, object]]] = {}
        for port_id in ports:
            if current_info_counts.get(port_id, 0) > before_info_counts.get(port_id, 0):
                results[port_id] = current_info_frames.get(port_id)
            else:
                results[port_id] = None
        return results

    def stop_auto_report(self, ports: Sequence[int]) -> None:
        if not ports:
            raise ValueError("no ports selected for quiet")
        self.send_protocol_command(0x10, bytes([1]), dev_id=0x00, ports=ports)

    def last_active_ports(self) -> List[int]:
        with self.state_lock:
            return list(self.last_scan_active_ports)

    def snapshot_hex_dashboard(self, ports: Sequence[int]) -> Dict[int, Dict[str, object]]:
        now = time.monotonic()
        activity_counts = self.snapshot_bridge_activity_counts()
        last_seen_times = self.snapshot_bridge_last_seen_times()
        latest_sensor_windows = self.snapshot_bridge_latest_sensor_windows()
        recent_streams = self.snapshot_bridge_recent_streams()
        total_bytes = self.snapshot_bridge_total_bytes()
        rows: Dict[int, Dict[str, object]] = {}

        for port_id in ports:
            last_seen = last_seen_times.get(port_id)
            rows[port_id] = {
                "active": last_seen is not None and (now - last_seen) <= 1.0,
                "frames": activity_counts.get(port_id, 0),
                "bytes": total_bytes.get(port_id, 0),
                "age_ms": None if last_seen is None else int((now - last_seen) * 1000.0),
                "payload": latest_sensor_windows.get(port_id, b""),
                "raw_tail": recent_streams.get(port_id, b"")[-LIVE_HEX_RAW_TAIL_BYTES:],
            }

        return rows


class TactileCli(cmd.Cmd):
    intro = (
        "tactile skin cli\n"
        "输入 help 查看命令。推荐先用 ports、scan、quiet。\n"
        "说明：0x10/0x11/0x12/0x13 的 payload 细节在文档里不完整，CLI 对 config/mode/algo 采用单字节假设，"
        "复杂命令请用 send 或 txraw。"
    )
    prompt = "cli> "

    def __init__(self, session: TactileSession) -> None:
        super().__init__()
        self.session = session

    def emptyline(self) -> bool:
        return False

    def default(self, line: str) -> None:
        if self._dispatch_symbolic_command(line):
            return
        super().default(line)

    @staticmethod
    def _format_hex_payload(payload: bytes) -> str:
        if not payload:
            return "-"
        return payload.hex(" ")

    def _dispatch_symbolic_command(self, line: str) -> bool:
        parts = line.strip().split()
        if len(parts) < 2:
            return False

        try:
            ports = parse_port_list(parts[0])
        except Exception:
            return False

        cmd_code = command_code_from_name(parts[1])
        if cmd_code is None:
            return False

        payload = encode_named_command_payload(cmd_code, parts[2:])
        self.session.send_protocol_command(cmd_code, payload, ports=ports)
        frame = build_sensor_frame(self.session.default_dev_id, cmd_code, payload)
        print(
            f"sent {','.join(port_label(p) for p in ports)} {parts[1].upper()} "
            f"-> {frame.hex(' ')}"
        )
        return True

    def do_ports(self, arg: str) -> None:
        """查看端口映射、当前 target/watch 状态。"""
        print(self.session.describe_ports())

    def do_target(self, arg: str) -> None:
        """设置发送目标端口。例如：target 0 1 2 或 target USART2,USART11 或 target all"""
        if not arg.strip():
            print(','.join(port_label(p) for p in self.session.target_ports) or '<none>')
            return
        self.session.target_ports = parse_port_list(arg)
        print(f"target -> {','.join(port_label(p) for p in self.session.target_ports) or '<none>'}")

    def do_watch(self, arg: str) -> None:
        """设置异步显示的端口过滤。例如：watch all / watch none / watch 6 10"""
        if not arg.strip():
            print(','.join(port_label(p) for p in sorted(self.session.watch_ports)) or '<none>')
            return
        self.session.watch_ports = set(parse_port_list(arg))
        print(f"watch -> {','.join(port_label(p) for p in sorted(self.session.watch_ports)) or '<none>'}")

    def do_monitor(self, arg: str) -> None:
        """设置显示模式：monitor summary|raw|quiet"""
        mode = arg.strip().lower()
        if mode not in MONITOR_MODES:
            print(f"monitor modes: {', '.join(sorted(MONITOR_MODES))}")
            return
        self.session.monitor = mode
        print(f"monitor -> {mode}")

    def do_devid(self, arg: str) -> None:
        """查看或设置默认传感器 DEV_ID。例如：devid 0x01"""
        if not arg.strip():
            print(f"0x{self.session.default_dev_id:02X}")
            return
        self.session.default_dev_id = int(arg.strip(), 0) & 0xFF
        print(f"default_dev_id -> 0x{self.session.default_dev_id:02X}")

    def do_info(self, arg: str) -> None:
        """查询设备信息。用法：info 或 info all / info USART2 USART11"""
        ports = self.session.target_ports if not arg.strip() else parse_port_list(arg)
        if not ports:
            raise ValueError("no target ports selected")
        results = self.session.query_device_info(ports)
        for port_id in ports:
            info = results.get(port_id)
            if info is None:
                print(f"{port_label(port_id)} -> no response")
                continue
            payload = info["device_info"]
            mode_name = MODE_NAMES.get(payload.get("out_mode"), payload.get("out_mode"))
            algo_name = ALGO_NAMES.get(payload.get("algo"), payload.get("algo"))
            print(
                f"{port_label(port_id)} -> "
                f"sn={payload.get('sn')} version={payload.get('version')} "
                f"rows={payload.get('rows')} cols={payload.get('cols')} "
                f"freq={payload.get('freq')}Hz mode={mode_name} algo={algo_name} "
                f"shear={payload.get('shear_enabled')}"
            )

    def do_cmds(self, arg: str) -> None:
        """列出支持的符号命令名和示例。"""
        print("支持的符号命令：")
        for code in sorted(CMD_NAMES):
            print(f"0x{code:02X} {CMD_NAMES[code]} / CMD_{CMD_NAMES[code]}")
        print("示例：")
        print("UART7 CMD_REBOOT")
        print("USART2 CMD_DEVICE_INFO")
        print("USART11 CMD_CONFIG_MODE stop")
        print("UART12 CMD_SET_MODE combined")
        print("USART10 CMD_SET_ALGO cnn")
        print("USART2 CMD_SET_FREQ 50")
        print("UART7 CMD_SET_SHEAR 01 00 02 00 03 00 01")

    def do_scan(self, arg: str) -> None:
        """扫描哪些端口有活动数据。用法：scan / scan all / scan USART2 USART11"""
        ports = self.session.target_ports if not arg.strip() else parse_port_list(arg)
        if not ports:
            raise ValueError("no target ports selected")
        results = self.session.observe_ports(ports)
        active_count = 0
        for port_id in ports:
            result = results[port_id]
            if not result["has_activity"]:
                print(f"{port_label(port_id)} -> no response")
                continue
            active_count += 1
            print(f"{port_label(port_id)} -> active")
        print(f"scan done: active {active_count}/{len(ports)}")

    def do_livehex(self, arg: str) -> None:
        """实时显示多端口十六进制报文。用法：livehex / livehex all / livehex UART7 USART2"""
        ports = self.session.target_ports if not arg.strip() else parse_port_list(arg)
        if not ports:
            raise ValueError("no target ports selected")

        print("按 Ctrl-C 退出 livehex。建议先执行 `monitor quiet`。")
        try:
            while True:
                rows = self.session.snapshot_hex_dashboard(ports)
                lines = [
                    "\x1b[2J\x1b[H",
                    "tactile live hex",
                    f"time={time.strftime('%H:%M:%S')} ports={','.join(port_label(p) for p in ports)}",
                    "port      state   frames    bytes   age_ms   latest_hex_full            raw_tail",
                    "--------------------------------------------------------------------------------",
                ]
                for port_id in ports:
                    row = rows[port_id]
                    state = "active" if row["active"] else "silent"
                    age_text = "-" if row["age_ms"] is None else str(row["age_ms"])
                    payload_text = self._format_hex_payload(row["payload"])
                    raw_tail_text = self._format_hex_payload(row["raw_tail"])
                    lines.append(
                        f"{port_label(port_id):<8}  "
                        f"{state:<6}  "
                        f"{row['frames']:<8}  "
                        f"{row['bytes']:<7}  "
                        f"{age_text:<6}  "
                        f"{payload_text:<28}  "
                        f"{raw_tail_text}"
                    )

                sys.stdout.write("\n".join(lines) + "\n")
                sys.stdout.flush()
                time.sleep(LIVE_HEX_REFRESH_S)
        except KeyboardInterrupt:
            print("\nlivehex stopped")

    def do_quiet(self, arg: str) -> None:
        """停止上次 scan 里活跃端口的自动上报，然后重新查询。"""
        ports = self.session.last_active_ports() if not arg.strip() else parse_port_list(arg)
        if not ports:
            print("no active ports from last scan")
            return
        self.session.stop_auto_report(ports)
        time.sleep(0.1)
        results = self.session.query_device_info(ports)
        for port_id in ports:
            info = results.get(port_id)
            if info is None:
                print(f"{port_label(port_id)} -> quiet sent, no query response")
                continue
            payload = info["device_info"]
            print(
                f"{port_label(port_id)} -> quiet ok "
                f"sn={payload.get('sn')} rows={payload.get('rows')} cols={payload.get('cols')}"
            )

    def do_discover(self, arg: str) -> None:
        """发送 discover。"""
        self.session.send_protocol_command(0x01)

    def do_freq(self, arg: str) -> None:
        """设置输出频率。用法：freq 50"""
        hz = int(arg.strip(), 0)
        if not 50 <= hz <= 100:
            raise ValueError("freq must be 50..100 Hz")
        self.session.send_protocol_command(0x14, hz.to_bytes(2, 'little', signed=False))
        print(f"sent CMD_SET_FREQ -> {hz} Hz")

    def do_algo(self, arg: str) -> None:
        """设置法向力算法。用法：algo poly|cnn|0|1。注意：这里假设 payload 为单字节。"""
        key = arg.strip().lower()
        mapping = {"poly": 0, "cnn": 1, "0": 0, "1": 1}
        if key not in mapping:
            raise ValueError("algo expects poly|cnn|0|1")
        value = mapping[key]
        self.session.send_protocol_command(0x13, bytes([value]))
        print(f"sent CMD_SET_ALGO -> {ALGO_NAMES[value]}")

    def do_mode(self, arg: str) -> None:
        """设置输出模式。用法：mode adc|force|combined|0|1|2。注意：这里假设 payload 为单字节。"""
        key = arg.strip().lower()
        mapping = {"adc": 0, "force": 1, "combined": 2, "0": 0, "1": 1, "2": 2}
        if key not in mapping:
            raise ValueError("mode expects adc|force|combined|0|1|2")
        value = mapping[key]
        self.session.send_protocol_command(0x11, bytes([value]))
        print(f"sent CMD_SET_MODE -> {MODE_NAMES[value]}")

    def do_config(self, arg: str) -> None:
        """进入/退出配置模式。用法：config enter|exit|0|1。注意：这里假设 payload 为单字节，enter=1, exit=0。"""
        key = arg.strip().lower()
        mapping = {"exit": 0, "enter": 1, "0": 0, "1": 1}
        if key not in mapping:
            raise ValueError("config expects enter|exit|0|1")
        value = mapping[key]
        self.session.send_protocol_command(0x10, bytes([value]))
        print(f"sent CMD_CONFIG_MODE -> {value}")

    def do_reboot(self, arg: str) -> None:
        """设备重启。注意：这里假设 reboot 无 payload。"""
        self.session.send_protocol_command(0x50)
        print("sent CMD_REBOOT")

    def do_send(self, arg: str) -> None:
        """通用协议命令。用法：send <cmd_hex> [payload_hex]；或 send UART7 CMD_REBOOT"""
        if self._dispatch_symbolic_command(arg):
            return
        parts = arg.strip().split(maxsplit=1)
        if not parts:
            raise ValueError("send expects at least cmd_hex")
        cmd_code = int(parts[0], 16)
        payload = parse_hex_payload(parts[1]) if len(parts) > 1 else b""
        self.session.send_protocol_command(cmd_code, payload)
        print(f"sent CMD 0x{cmd_code:02X} payload={payload.hex(' ')}")

    def do_txraw(self, arg: str) -> None:
        """直接发原始 UART 字节，不自动包传感器协议帧。用法：txraw A5 01 02 00 00 xx xx 5A"""
        payload = parse_hex_payload(arg)
        if not payload:
            raise ValueError("txraw expects bytes")
        self.session.send_sensor_frame(payload)
        print(f"sent raw uart payload={payload.hex(' ')}")

    def do_show(self, arg: str) -> None:
        """查看某一路最近一帧。用法：show 6 或 show USART2 raw"""
        parts = arg.strip().split(maxsplit=1)
        if not parts:
            raise ValueError("show expects a port")
        port_id = parse_port_list(parts[0])[0]
        frame = self.session.snapshot_frames().get(port_id)
        if frame is None:
            print(f"[{port_label(port_id)}] no parsed frame yet")
            return
        if len(parts) > 1 and parts[1].strip().lower() == "raw":
            print(frame.raw.hex(' '))
            return
        print(json.dumps(decode_sensor_frame(frame), ensure_ascii=False, indent=2))

    def do_sleep(self, arg: str) -> None:
        """暂停一段时间，方便看流。用法：sleep 2"""
        delay = float(arg.strip())
        time.sleep(delay)

    def do_quit(self, arg: str) -> bool:
        """退出 CLI。"""
        return True

    def do_exit(self, arg: str) -> bool:
        """退出 CLI。"""
        return True

    def do_EOF(self, arg: str) -> bool:
        print()
        return True


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Interactive CLI for tactile skin configuration over tactile_h562 CDC bridge")
    parser.add_argument("--port", default=pick_default_port(), help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="host-side serial baudrate placeholder for CDC ACM")
    parser.add_argument("--dev-id", type=lambda s: int(s, 0), default=DEFAULT_DEV_ID, help="default sensor DEV_ID")
    parser.add_argument("--target", default="all", help="default target ports, e.g. '0', '6 10', 'all'")
    parser.add_argument("--monitor", default="summary", choices=sorted(MONITOR_MODES), help="async display mode")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if not args.port:
        print("No serial port found. Use --port /dev/ttyACM0", file=sys.stderr)
        return 1

    target_ports = parse_port_list(args.target)
    with open_serial(args.port, args.baud, timeout=0.05) as ser:
        ser.reset_input_buffer()
        time.sleep(0.2)
        session = TactileSession(ser, args.dev_id & 0xFF, target_ports, args.monitor)
        session.start()
        try:
            TactileCli(session).cmdloop()
        finally:
            session.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

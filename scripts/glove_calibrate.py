#!/usr/bin/env python3
import argparse
import json
import os
import re
import sys
import time
import threading

import serial
from serial.tools import list_ports


FRAME_RE = re.compile(
    r"(I2C[12])@0x([0-9A-Fa-f]{2})\[A0:(-?\d+),A1:(-?\d+),A2:(-?\d+),A3:(-?\d+)\]"
)

COUNTS_PER_TURN = 24370.0
HALF_TURN = COUNTS_PER_TURN / 2.0
DEFAULT_RATE = COUNTS_PER_TURN / 360.0
MIN_VALID_DELTA = 30.0

JOINTS = [
    ("thumb.DIP", "I2C1", 0x48, 0),
    ("thumb.PIP", "I2C1", 0x48, 1),
    ("thumb.MCP_front", "I2C1", 0x48, 2),
    ("thumb.MCP_back", "I2C1", 0x48, 3),
    ("index.DIP", "I2C1", 0x49, 0),
    ("index.PIP", "I2C1", 0x49, 1),
    ("index.MCP_front", "I2C1", 0x49, 2),
    ("index.MCP_back", "I2C1", 0x49, 3),
    ("middle.DIP", "I2C2", 0x48, 0),
    ("middle.PIP", "I2C2", 0x48, 1),
    ("middle.MCP_front", "I2C2", 0x48, 2),
    ("middle.MCP_back", "I2C2", 0x48, 3),
    ("ring.DIP", "I2C2", 0x49, 0),
    ("ring.PIP", "I2C2", 0x49, 1),
    ("ring.MCP_front", "I2C2", 0x49, 2),
    ("ring.MCP_back", "I2C2", 0x49, 3),
    ("pinky.DIP", "I2C2", 0x4B, 0),
    ("pinky.PIP", "I2C2", 0x4B, 1),
    ("pinky.MCP_front", "I2C2", 0x4B, 2),
    ("pinky.MCP_back", "I2C2", 0x4B, 3),
]
EXPECTED_KEYS = {f"{bus}@0x{addr:02X}" for _, bus, addr, _ in JOINTS}


def pick_default_port():
    ports = [port.device for port in list_ports.comports()]
    if not ports:
        return ""
    for port in ports:
        if "ttyACM" in port or "ttyUSB" in port:
            return port
    return ports[0]


def parse_frame(line):
    frame = {}
    for match in FRAME_RE.finditer(line):
        key = f"{match.group(1)}@0x{int(match.group(2), 16):02X}"
        if key in frame:
            return None
        frame[key] = [
            int(match.group(3)),
            int(match.group(4)),
            int(match.group(5)),
            int(match.group(6)),
        ]
    if not frame:
        return None
    if set(frame.keys()) != EXPECTED_KEYS:
        return None
    return frame


def shortest_delta(new_value, ref_value):
    delta = float(new_value) - float(ref_value)
    if delta > HALF_TURN:
        delta -= COUNTS_PER_TURN
    elif delta < -HALF_TURN:
        delta += COUNTS_PER_TURN
    return delta


class LatestFrameCache:
    def __init__(self, ser):
        self.ser = ser
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_line = ""
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        while self.running:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            frame = parse_frame(line)
            if frame is None:
                continue
            with self.lock:
                self.latest_frame = frame
                self.latest_line = line

    def snapshot(self):
        with self.lock:
            if self.latest_frame is None:
                return None, ""
            return dict(self.latest_frame), self.latest_line

    def close(self):
        self.running = False
        self.thread.join(timeout=0.5)


def capture_one_value(cache, bus, addr, channel):
    sensor_key = f"{bus}@0x{addr:02X}"
    frame, line = cache.snapshot()
    if frame is None:
        return None, line, "no full frame available yet"
    if sensor_key not in frame:
        return None, line, f"missing {sensor_key}"
    return float(frame[sensor_key][channel]), line, None


def capture_on_enter(cache, bus, addr, channel, label):
    while True:
        input(f"Set {label}, press Enter: ")
        value, line, error = capture_one_value(cache, bus, addr, channel)
        if error is None:
            print(f"{label}={value:.0f}")
            print(f"frame: {line}")
            return value
        print(f"capture failed: {error}")
        print(f"raw: {line}")


def is_done(entry):
    return isinstance(entry, dict) and "rate" in entry and "d0" in entry


def is_custom_mcp_back(name):
    return name.endswith("MCP_back") and not name.startswith("thumb.")


def load_calibration(path):
    if not os.path.exists(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        if isinstance(data, dict):
            return data
    except Exception:
        pass
    return {}


def build_entry(name, bus, addr, channel, d0, d_ref, angle_ref):
    delta = shortest_delta(d_ref, d0)
    if abs(delta) < MIN_VALID_DELTA:
        return None, f"{name}: |d_ref-d0|={abs(delta):.3f} too small"

    rate = delta / float(angle_ref)
    k = float(angle_ref) / delta
    b = -k * d0

    if abs(rate) < 20.0 or abs(rate) > 200.0:
        print(
            f"warning: {name} abnormal rate={rate:.6f}, default={DEFAULT_RATE:.6f}",
            file=sys.stderr,
        )

    return {
        "sensor": f"{bus}@0x{addr:02X}",
        "channel": channel,
        "angle0": 0.0,
        "angle_ref": float(angle_ref),
        "d0": float(d0),
        "d_ref": float(d_ref),
        "offset": float(d0),
        "rate": float(rate),
        "k": float(k),
        "b": float(b),
    }, None


def main():
    default_out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glove_calibration.json")
    parser = argparse.ArgumentParser(description="Simple Enter->capture calibration.")
    parser.add_argument("--port", default=pick_default_port(), help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--out", default=default_out, help="Output JSON")
    args = parser.parse_args()

    if not args.port:
        print("No serial port found. Use --port /dev/ttyACM0", file=sys.stderr)
        return 1

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    calibration = load_calibration(args.out)
    done = sum(1 for name, _, _, _ in JOINTS if is_done(calibration.get(name)))
    if done > 0:
        print(f"resume mode: {done}/20 joints already calibrated", file=sys.stderr)
    print(f"Opening {args.port} @ {args.baud}", file=sys.stderr)

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        time.sleep(0.2)
        cache = LatestFrameCache(ser)
        try:
            time.sleep(0.4)
            for index, (name, bus, addr, channel) in enumerate(JOINTS, start=1):
                if is_done(calibration.get(name)):
                    print(f"\n[{index:02d}/20] {name} already done, skip.")
                    continue
                print(f"\n[{index:02d}/20] {name} <- {bus}@0x{addr:02X} A{channel}")
                d0 = capture_on_enter(cache, bus, addr, channel, "0deg")

                if is_custom_mcp_back(name):
                    while True:
                        mode = input(
                            "MCP_back ref: enter angle number (e.g. 70), Enter for default 90 capture, or 'skip' to use default ratio: "
                        ).strip()
                        if mode.lower() == "skip":
                            entry = {
                                "sensor": f"{bus}@0x{addr:02X}",
                                "channel": channel,
                                "angle0": 0.0,
                                "angle_ref": 90.0,
                                "d0": float(d0),
                                "d_ref": None,
                                "offset": float(d0),
                                "rate": float(DEFAULT_RATE),
                                "k": float(1.0 / DEFAULT_RATE),
                                "b": float(-d0 / DEFAULT_RATE),
                            }
                            calibration[name] = entry
                            print(f"{name}: using default ratio only (no 90deg capture).")
                            break

                        if mode == "":
                            angle_ref = 90.0
                        else:
                            try:
                                angle_ref = float(mode)
                            except ValueError:
                                print("invalid angle input, retry")
                                continue
                            if abs(angle_ref) < 1e-9:
                                print("angle cannot be zero, retry")
                                continue

                        while True:
                            d_ref = capture_on_enter(cache, bus, addr, channel, f"{angle_ref:.3f}deg")
                            entry, error = build_entry(name, bus, addr, channel, d0, d_ref, angle_ref)
                            if error is None:
                                calibration[name] = entry
                                break
                            print(error)
                            print("Re-capture reference angle.")
                        break
                else:
                    while True:
                        d90 = capture_on_enter(cache, bus, addr, channel, "90deg")
                        entry, error = build_entry(name, bus, addr, channel, d0, d90, 90.0)
                        if error is None:
                            calibration[name] = entry
                            break
                        print(error)
                        print("Re-capture only 90deg.")

                with open(args.out, "w", encoding="utf-8") as handle:
                    json.dump(calibration, handle, indent=2, ensure_ascii=True)
                print(f"saved -> {args.out}")
        finally:
            cache.close()

    print(f"\nCalibration complete: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

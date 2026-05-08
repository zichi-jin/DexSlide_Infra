#!/usr/bin/env python3
import argparse
import json
import re
import sys
import time

import serial
from serial.tools import list_ports


FRAME_RE = re.compile(
    r"(I2C[12])@0x([0-9A-Fa-f]{2})\[A0:(-?\d+),A1:(-?\d+),A2:(-?\d+),A3:(-?\d+)\]"
)
COUNTS_PER_TURN = 24370.0
DEFAULT_RATE_COUNT_PER_DEG = COUNTS_PER_TURN / 360.0
WRAP_THRESHOLD = COUNTS_PER_TURN / 2.0
JOINT_LABELS = ("DIP", "PIP", "MCP_front", "MCP_back")
SENSOR_ORDER = (
    ("I2C1", 0x48, "thumb"),
    ("I2C1", 0x49, "index"),
    ("I2C2", 0x48, "middle"),
    ("I2C2", 0x49, "ring"),
    ("I2C2", 0x4B, "pinky"),
)


def parse_frame(line: str):
    out = []
    for m in FRAME_RE.finditer(line):
        out.append(
            {
                "bus": m.group(1),
                "address": int(m.group(2), 16),
                "a0": int(m.group(3)),
                "a1": int(m.group(4)),
                "a2": int(m.group(5)),
                "a3": int(m.group(6)),
            }
        )
    return out


def records_to_sensor_map(records):
    sensor_map = {}
    for rec in records:
        key = f"{rec['bus']}@0x{rec['address']:02X}"
        sensor_map[key] = [rec["a0"], rec["a1"], rec["a2"], rec["a3"]]
    return sensor_map


def make_joint_order():
    order = []
    for bus, addr, finger in SENSOR_ORDER:
        sensor_key = f"{bus}@0x{addr:02X}"
        for channel_idx, joint_name in enumerate(JOINT_LABELS):
            order.append(
                {
                    "id": f"{finger}.{joint_name}",
                    "finger": finger,
                    "joint": joint_name,
                    "sensor": sensor_key,
                    "channel": channel_idx,
                }
            )
    return order


def pick_default_port() -> str:
    ports = [p.device for p in list_ports.comports()]
    if not ports:
        return ""
    for cand in ports:
        if "ttyACM" in cand or "ttyUSB" in cand:
            return cand
    return ports[0]


def read_frame_line(ser):
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        if line:
            return line


def wait_parsed_frame(ser):
    while True:
        line = read_frame_line(ser)
        records = parse_frame(line)
        if records:
            return records, line


def normalize_delta(delta):
    if delta > WRAP_THRESHOLD:
        return delta - COUNTS_PER_TURN
    if delta < -WRAP_THRESHOLD:
        return delta + COUNTS_PER_TURN
    return delta


def build_default_calibration(joint_order):
    calibration = {}
    for joint_def in joint_order:
        calibration[joint_def["id"]] = {
            "rate": DEFAULT_RATE_COUNT_PER_DEG,
            "angle0": 0.0,
            "offset": 0.0,
        }
    return calibration


def load_calibration(path, joint_order):
    if path is None:
        return build_default_calibration(joint_order)
    with open(path, "r", encoding="utf-8") as handle:
        calib = json.load(handle)
    out = build_default_calibration(joint_order)
    out.update(calib)
    return out


def save_calibration(path, calib):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(calib, handle, indent=2, ensure_ascii=True)


def run_angle_live(ser, joint_order, calib, as_json):
    runtime = {}
    for joint_def in joint_order:
        conf = calib.get(joint_def["id"], {})
        rate = float(conf.get("rate", 0.0))
        if abs(rate) < 1e-9:
            k_old = float(conf.get("k", 0.0))
            if abs(k_old) > 1e-9:
                rate = 1.0 / k_old
            else:
                rate = DEFAULT_RATE_COUNT_PER_DEG
        if abs(rate) < 20.0 or abs(rate) > 200.0:
            sign = -1.0 if rate < 0.0 else 1.0
            rate = sign * DEFAULT_RATE_COUNT_PER_DEG
            print(
                f"warning: {joint_def['id']} rate abnormal, fallback to {rate:.3f} count/deg",
                file=sys.stderr,
            )
        runtime[joint_def["id"]] = {
            "offset": float(conf.get("offset", conf.get("d0", 0.0))),
            "rate": rate,
            "angle0": float(conf.get("angle0", 0.0)),
        }

    while True:
        records, raw_line = wait_parsed_frame(ser)
        sensor_map = records_to_sensor_map(records)
        payload = {}
        missing = []
        for joint_def in joint_order:
            key = joint_def["id"]
            sensor_values = sensor_map.get(joint_def["sensor"])
            if sensor_values is None:
                missing.append(joint_def["sensor"])
                continue
            raw_count = float(sensor_values[joint_def["channel"]])
            state = runtime[key]
            angle = state["angle0"] + (
                normalize_delta(raw_count - state["offset"]) / state["rate"]
            )
            payload[key] = angle

        if as_json:
            out = {
                "angles_deg": payload,
                "missing_sensors": sorted(set(missing)),
            }
            print(json.dumps(out, ensure_ascii=True))
        else:
            if not payload:
                print("No mapped sensors in frame:", raw_line)
                continue
            joined = ", ".join(f"{name}:{value:.2f}" for name, value in payload.items())
            print(joined)


def main():
    parser = argparse.ArgumentParser(description="Monitor ADS1115 live stream from STM32 over serial.")
    parser.add_argument("--port", default=pick_default_port(), help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument("--json", action="store_true", help="Print parsed frames as JSON")
    parser.add_argument("--angles", action="store_true", help="Print 20 calibrated joint angles")
    parser.add_argument("--calib-file", default="glove_calibration.json", help="Calibration json path")
    args = parser.parse_args()

    if not args.port:
        print("No serial port found. Try --port /dev/ttyACM0", file=sys.stderr)
        return 1

    print(f"Opening {args.port} @ {args.baud} ...", file=sys.stderr)
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        time.sleep(0.2)
        joint_order = make_joint_order()

        if args.angles:
            calib = load_calibration(args.calib_file, joint_order)
            run_angle_live(ser, joint_order, calib, as_json=args.json)
            return 0

        while True:
            line = read_frame_line(ser)
            if args.json:
                frame = parse_frame(line)
                if frame:
                    print(json.dumps(frame, ensure_ascii=True))
                else:
                    print(json.dumps({"raw": line}, ensure_ascii=True))
            else:
                print(line)


if __name__ == "__main__":
    raise SystemExit(main())

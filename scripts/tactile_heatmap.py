#!/usr/bin/env python3
"""Display the 11 STM32H562 tactile sensors as live 12x8 heatmaps."""

from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from tactile_api import SensorFrameEvent, StatusEvent, TactileCdcDecoder, TactileMeasurement
from tactile_stream import PORT_LABELS, choose_default_port


CHANNELS = ("auto", "adc", "normal", "shear-x", "shear-y")
DEFAULT_ROWS = 12
DEFAULT_COLS = 8
READ_CHUNK_BYTES = 64 * 1024
TITLE_REFRESH_INTERVAL_S = 0.25
DEFAULT_LABEL_REFRESH_INTERVAL_S = 0.20


@dataclass(frozen=True)
class LiveFrame:
    """The latest decodable matrix for one tactile connector."""

    matrix: np.ndarray
    channel_label: str
    sequence: int
    received_at: float


def select_measurement_matrix(
    measurement: TactileMeasurement,
    channel: str,
) -> Optional[tuple[tuple[tuple[Optional[int], ...], ...], str]]:
    """Select one matrix from a decoded measurement for display.

    ``auto`` prefers ADC data and otherwise displays normal force. It therefore
    supports the current ADC reporting mode as well as a future COMBINED mode.
    """

    choices = {
        "adc": (measurement.adc_counts, "ADC counts"),
        "normal": (measurement.normal_force_mn, "normal force (mN)"),
        "shear-x": (measurement.shear_x_mn, "shear X (mN)"),
        "shear-y": (measurement.shear_y_mn, "shear Y (mN)"),
    }
    if channel == "auto":
        for candidate in ("adc", "normal"):
            matrix, label = choices[candidate]
            if matrix is not None:
                return matrix, label
        return None
    matrix, label = choices[channel]
    return None if matrix is None else (matrix, label)


def matrix_to_array(matrix: tuple[tuple[Optional[int], ...], ...]) -> np.ndarray:
    """Convert a protocol matrix to an image array, masking invalid sensor cells."""

    return np.array(
        [[np.nan if value is None else value for value in row] for row in matrix],
        dtype=float,
    )


def automatic_color_limits(arrays: Iterable[np.ndarray]) -> Optional[tuple[float, float]]:
    """Return global limits covering every valid value in ``arrays``."""

    valid_values = [array[np.isfinite(array)] for array in arrays]
    valid_values = [values for values in valid_values if values.size]
    if not valid_values:
        return None

    minimum = float(min(np.min(values) for values in valid_values))
    maximum = float(max(np.max(values) for values in valid_values))
    if minimum == maximum:
        return (minimum - 1.0, maximum + 1.0)

    padding = (maximum - minimum) * 0.02
    return (minimum - padding, maximum + padding)


def matrix_statistics(array: np.ndarray) -> Optional[tuple[int, int, int]]:
    """Return minimum, maximum, and nonzero-cell count for valid sensor cells."""

    values = array[np.isfinite(array)]
    if not values.size:
        return None
    return int(np.min(values)), int(np.max(values)), int(np.count_nonzero(values))


def format_cell_value(value: float) -> str:
    """Format one valid image value without hiding its actual protocol value."""

    if not np.isfinite(value):
        return "—"
    return str(int(value)) if float(value).is_integer() else f"{value:.1f}"


class TactileHeatmap:
    """Own the thread-safe decoder state and matplotlib artists for the live view."""

    def __init__(
        self,
        channel: str,
        vmin: Optional[float],
        vmax: Optional[float],
        annotate: bool = False,
        annotate_nonzero_only: bool = True,
        label_refresh_interval_s: float = DEFAULT_LABEL_REFRESH_INTERVAL_S,
    ) -> None:
        self.channel = channel
        self.vmin = vmin
        self.vmax = vmax
        self.annotate = annotate
        self.annotate_nonzero_only = annotate_nonzero_only
        self.label_refresh_interval_s = label_refresh_interval_s
        self.decoder = TactileCdcDecoder()
        self.frames: dict[int, LiveFrame] = {}
        self.latest_status = None
        self.ignored_measurements = 0
        self._state_lock = threading.Lock()
        self._color_limits = (0.0, 1.0)
        self._images = []
        self._axes = []
        self._title_texts = []
        self._value_texts = []
        self._status_axis = None
        self._status_text = None
        self._colorbar = None
        self._figure = None
        self._rendered_sequences: dict[int, int] = {}
        self._labelled_sequences: dict[int, int] = {}
        self._last_title_update = 0.0
        self._last_label_update = 0.0

    def feed(self, chunk: bytes) -> None:
        """Decode one arbitrary CDC chunk and retain its newest displayable frames."""

        received_at = time.monotonic()
        with self._state_lock:
            for event in self.decoder.feed(chunk):
                if isinstance(event, StatusEvent):
                    self.latest_status = event.status
                    continue
                if not isinstance(event, SensorFrameEvent) or event.measurement is None:
                    continue
                selected = select_measurement_matrix(event.measurement, self.channel)
                if selected is None:
                    self.ignored_measurements += 1
                    continue
                matrix, channel_label = selected
                self.frames[event.port_id] = LiveFrame(
                    matrix=matrix_to_array(matrix),
                    channel_label=channel_label,
                    sequence=event.measurement.seq,
                    received_at=received_at,
                )

    def drain_serial(self, serial_port, budget: int = READ_CHUNK_BYTES) -> int:
        """Read all currently queued CDC bytes, up to ``budget``, without blocking."""

        total = 0
        while total < budget:
            waiting = int(getattr(serial_port, "in_waiting", 0))
            if waiting <= 0:
                break
            chunk = serial_port.read(min(waiting, budget - total))
            if not chunk:
                break
            self.feed(chunk)
            total += len(chunk)
        return total

    def configure_color_limits(self) -> tuple[float, float]:
        """Freeze a shared colour range from the currently received raw values."""

        frames, _, _ = self._snapshot()
        automatic_limits = automatic_color_limits(frame.matrix for frame in frames.values())
        lower = self.vmin if self.vmin is not None else (automatic_limits[0] if automatic_limits else 0.0)
        upper = self.vmax if self.vmax is not None else (automatic_limits[1] if automatic_limits else 1.0)
        if lower >= upper:
            upper = lower + 1.0
        self._color_limits = (lower, upper)

        if self._figure is not None:
            for image in self._images:
                image.set_clim(lower, upper)
            if self._colorbar is not None:
                self._colorbar.update_normal(self._images[0])
            self._figure.canvas.draw_idle()
        return self._color_limits

    def create_figure(self) -> None:
        """Create a fixed-layout dashboard suitable for fast blitted redraws."""

        cmap = plt.get_cmap("coolwarm" if self.channel.startswith("shear") else "viridis").copy()
        cmap.set_bad(color="#d9d9d9")
        lower, upper = self._color_limits
        figure, axes = plt.subplots(3, 4, figsize=(15, 11))
        figure.subplots_adjust(left=0.055, right=0.89, bottom=0.06, top=0.88, wspace=0.34, hspace=0.52)
        self._figure = figure
        self._axes = list(axes.flat)
        for port_id, axis in enumerate(self._axes):
            if port_id not in PORT_LABELS:
                axis.set_visible(False)
                continue
            image = axis.imshow(
                np.full((DEFAULT_ROWS, DEFAULT_COLS), np.nan),
                cmap=cmap,
                vmin=lower,
                vmax=upper,
                interpolation="nearest",
                origin="upper",
                aspect="equal",
            )
            self._images.append(image)
            self._title_texts.append(axis.set_title(f"{PORT_LABELS[port_id]}\nwaiting for data", fontsize=9, pad=5))
            value_texts = []
            if self.annotate:
                for row in range(DEFAULT_ROWS):
                    for col in range(DEFAULT_COLS):
                        value_texts.append(axis.text(
                            col,
                            row,
                            "",
                            ha="center",
                            va="center",
                            color="white",
                            fontsize=5.5,
                            clip_on=True,
                        ))
            self._value_texts.append(value_texts)
            axis.set_xlabel("column")
            axis.set_ylabel("row")
            axis.set_xticks(range(0, DEFAULT_COLS, 2))
            axis.set_yticks(range(0, DEFAULT_ROWS, 2))

        self._colorbar = figure.colorbar(
            self._images[0],
            ax=[axis for axis in self._axes if axis.get_visible()],
            shrink=0.88,
            label=self._colorbar_label(),
        )
        # Figure-level text has no Axes and cannot be used by FuncAnimation blitting.
        self._status_axis = figure.add_axes((0.055, 0.915, 0.835, 0.035), frameon=False)
        self._status_axis.set_axis_off()
        self._status_text = self._status_axis.text(
            0.5,
            0.5,
            self._status_title(0, 0, None),
            ha="center",
            va="center",
            fontsize=13,
        )

    def initial_artists(self) -> list:
        """Return every artist required for the initial FuncAnimation blit frame."""

        return [
            *self._images,
            *self._title_texts,
            self._status_text,
        ]

    def _colorbar_label(self) -> str:
        if self.channel == "auto":
            return "ADC counts / normal force (mN)"
        return {
            "adc": "ADC counts",
            "normal": "normal force (mN)",
            "shear-x": "shear X (mN)",
            "shear-y": "shear Y (mN)",
        }[self.channel]

    def render(self) -> list:
        """Update only image artists whose source port has supplied a new frame."""

        if self._figure is None or self._status_text is None:
            raise RuntimeError("create_figure() must be called before render()")

        now = time.monotonic()
        frames, status, transport_bad = self._snapshot()
        refresh_titles = (now - self._last_title_update) >= TITLE_REFRESH_INTERVAL_S
        refresh_labels = self.annotate and (now - self._last_label_update) >= self.label_refresh_interval_s
        artists = []
        for port_id, image in enumerate(self._images):
            live_frame = frames.get(port_id)
            if live_frame is not None and self._rendered_sequences.get(port_id) != live_frame.sequence:
                image.set_data(live_frame.matrix)
                self._rendered_sequences[port_id] = live_frame.sequence
                artists.append(image)
            if refresh_titles:
                if live_frame is None:
                    self._title_texts[port_id].set_text(f"{PORT_LABELS[port_id]}\nwaiting for data")
                else:
                    age_ms = (now - live_frame.received_at) * 1000.0
                    self._title_texts[port_id].set_text(self._frame_title(port_id, live_frame, age_ms))
                artists.append(self._title_texts[port_id])

            if refresh_labels and live_frame is not None:
                if self._labelled_sequences.get(port_id) != live_frame.sequence:
                    artists.extend(self._update_value_texts(port_id, live_frame.matrix))
                    self._labelled_sequences[port_id] = live_frame.sequence

        if refresh_titles:
            self._status_text.set_text(self._status_title(len(frames), transport_bad, status))
            artists.append(self._status_text)
            self._last_title_update = now
        if refresh_labels:
            self._last_label_update = now
        return artists

    def _update_value_texts(self, port_id: int, matrix: np.ndarray) -> list:
        lower, upper = self._color_limits
        artists = []
        for row in range(DEFAULT_ROWS):
            for col in range(DEFAULT_COLS):
                index = row * DEFAULT_COLS + col
                value = matrix[row, col] if row < matrix.shape[0] and col < matrix.shape[1] else np.nan
                text = self._value_texts[port_id][index]
                show_value = np.isfinite(value) and (not self.annotate_nonzero_only or value != 0)
                new_text = format_cell_value(value) if show_value else ""
                changed = text.get_text() != new_text
                if changed:
                    text.set_text(new_text)

                if show_value:
                    new_color = "white" if (upper > lower) and ((value - lower) / (upper - lower)) < 0.55 else "black"
                    if text.get_color() != new_color:
                        text.set_color(new_color)
                        changed = True
                if changed:
                    artists.append(text)
        return artists

    def _snapshot(self) -> tuple[dict[int, LiveFrame], object, int]:
        with self._state_lock:
            return dict(self.frames), self.latest_status, self.decoder.transport_bad_records

    def _frame_title(self, port_id: int, live_frame: LiveFrame, age_ms: float) -> str:
        statistics = matrix_statistics(live_frame.matrix)
        if statistics is None:
            value_text = "no valid cells"
        else:
            minimum, maximum, nonzero = statistics
            value_text = f"range={minimum}..{maximum}, nonzero={nonzero}"
        return (
            f"{PORT_LABELS[port_id]}\n"
            f"{live_frame.channel_label}, seq={live_frame.sequence}, age={age_ms:.0f} ms\n"
            f"{value_text}"
        )

    def _status_title(self, active_ports: int, transport_bad: int, status) -> str:
        lower, upper = self._color_limits
        title = (
            f"STM32H562 tactile heatmaps | channel={self.channel} | "
            f"active={active_ports}/{len(PORT_LABELS)} | range={lower:g}..{upper:g} | "
            f"transport_bad={transport_bad}"
        )
        if status is not None:
            pending = status.usb_tx_pending
            if pending is not None:
                title += f" | USB pending={pending} B"
            if status.status_drops is not None:
                title += f" | status_drop={status.status_drops}"
        return title


class TactileSerialReader:
    """Continuously decode CDC bytes so GUI drawing never throttles USB reading."""

    def __init__(self, serial_port, dashboard: TactileHeatmap) -> None:
        self._serial_port = serial_port
        self._dashboard = dashboard
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="tactile-cdc-reader", daemon=True)
        self.error: Optional[Exception] = None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        cancel_read = getattr(self._serial_port, "cancel_read", None)
        if cancel_read is not None:
            try:
                cancel_read()
            except OSError:
                pass
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        try:
            while not self._stop_event.is_set():
                waiting = int(getattr(self._serial_port, "in_waiting", 0))
                chunk = self._serial_port.read(min(max(waiting, 1), READ_CHUNK_BYTES))
                if chunk:
                    self._dashboard.feed(chunk)
        except Exception as exc:  # The main thread presents the serial error to the user.
            if not self._stop_event.is_set():
                self.error = exc


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Show 11 tactile sensors as live 12x8 matplotlib heatmaps")
    parser.add_argument("--port", default=choose_default_port(), help="CDC device, e.g. /dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200, help="CDC line-setting value")
    parser.add_argument(
        "--channel",
        choices=CHANNELS,
        default="auto",
        help="matrix to display; auto prefers ADC then normal force",
    )
    parser.add_argument("--interval-ms", type=int, default=16, help="GUI redraw interval, default: 16 ms")
    parser.add_argument(
        "--scale-warmup-ms",
        type=int,
        default=1000,
        help="collect data for this long before locking the shared colour range, default: 1000 ms",
    )
    parser.add_argument("--vmin", type=float, help="fixed lower colour limit")
    parser.add_argument("--vmax", type=float, help="fixed upper colour limit")
    parser.add_argument(
        "--annotate",
        action="store_true",
        help="draw latest nonzero raw values; zero cells remain unlabelled",
    )
    parser.add_argument(
        "--annotate-all",
        action="store_true",
        help="draw every raw value, including zero; significantly slower than --annotate",
    )
    parser.add_argument(
        "--label-interval-ms",
        type=int,
        default=int(DEFAULT_LABEL_REFRESH_INTERVAL_S * 1000),
        help="cell-number refresh interval when --annotate is used, default: 200 ms",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    try:
        import serial  # type: ignore
    except ModuleNotFoundError:
        print("需要 pyserial：python3 -m pip install pyserial", file=sys.stderr)
        return 2

    if not args.port:
        print("未找到串口，请用 --port 指定 /dev/ttyACM*", file=sys.stderr)
        return 2
    if args.interval_ms < 1 or args.scale_warmup_ms < 0 or args.label_interval_ms < 1:
        print("--interval-ms 与 --label-interval-ms 必须大于 0，--scale-warmup-ms 不能为负数", file=sys.stderr)
        return 2
    if args.vmin is not None and args.vmax is not None and args.vmin >= args.vmax:
        print("--vmin 必须小于 --vmax", file=sys.stderr)
        return 2

    dashboard = TactileHeatmap(
        args.channel,
        args.vmin,
        args.vmax,
        annotate=args.annotate or args.annotate_all,
        annotate_nonzero_only=not args.annotate_all,
        label_refresh_interval_s=args.label_interval_ms / 1000.0,
    )
    reader = None
    try:
        with serial.Serial(args.port, args.baud, timeout=0.02) as serial_port:
            serial_port.dtr = True
            reader = TactileSerialReader(serial_port, dashboard)
            reader.start()
            if args.vmin is None or args.vmax is None:
                print(f"collecting {args.scale_warmup_ms} ms of raw data to lock the shared colour range")
                deadline = time.monotonic() + args.scale_warmup_ms / 1000.0
                while time.monotonic() < deadline and reader.error is None:
                    time.sleep(0.01)
            lower, upper = dashboard.configure_color_limits()
            dashboard.create_figure()
            print(
                f"showing 11 tactile heatmaps from {args.port}; redraw interval={args.interval_ms} ms, "
                f"channel={args.channel}, fixed range={lower:g}..{upper:g}, "
                f"labels={'all' if args.annotate_all else 'nonzero' if args.annotate else 'off'}; press r to rescale"
            )
            reader_error_reported = False

            def update(_frame_number: int):
                nonlocal reader_error_reported
                if reader.error is not None and not reader_error_reported:
                    print(f"CDC reader stopped: {reader.error}", file=sys.stderr)
                    reader_error_reported = True
                return dashboard.render()

            def on_key(event) -> None:
                if event.key == "r":
                    lower, upper = dashboard.configure_color_limits()
                    print(f"shared colour range reset to {lower:g}..{upper:g}")

            dashboard._figure.canvas.mpl_connect("key_press_event", on_key)
            animation = FuncAnimation(
                dashboard._figure,
                update,
                init_func=dashboard.initial_artists,
                interval=args.interval_ms,
                blit=True,
                cache_frame_data=False,
            )
            # Keep a strong reference until the GUI window closes.
            _ = animation
            plt.show()
    except serial.SerialException as exc:
        print(f"无法打开或读取 {args.port}: {exc}", file=sys.stderr)
        return 2
    finally:
        if reader is not None:
            reader.stop()
    return 0


def main() -> int:
    return run(build_argument_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())

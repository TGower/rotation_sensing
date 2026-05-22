#!/usr/bin/env python3
"""Terminal UI for the Sender serial protocol."""

from __future__ import annotations

import argparse
import curses
import glob
import math
import os
import re
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Deque, Optional

try:
    import serial
except ImportError:  # pragma: no cover - exercised only on unprepared hosts.
    serial = None


START_BYTE_TX = 0xAA
START_BYTE_RX = 0xAB
ESCAPE_BYTE_TX = 0x7D
ESCAPE_XOR_TX = 0x20
ESCAPED_TX_BYTES = {START_BYTE_TX, ESCAPE_BYTE_TX, 10, 13}

APP_PACKET_TYPE_CONTROL = 0x10
APP_PACKET_TYPE_CONFIG_SET = 0x20
APP_PACKET_TYPE_CONFIG_STATE = 0x21
APP_PACKET_TYPE_STATS = 0x30
APP_PACKET_TYPE_CMD_DUMP = 0x40
APP_PACKET_TYPE_CMD_ACK = 0x41
APP_PROTOCOL_MAGIC = 164
APP_CMD_DUMP_PACKET_SIZE = 2
APP_CMD_ACK_PACKET_SIZE = 3

CONTROL_STRUCT = struct.Struct("<BBHff")
CONFIG_STRUCT = struct.Struct("<BBBBBBHHffHHfBB")
STATS_STRUCT = struct.Struct("<BBffiBfffI")

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
FRAME_RE = re.compile(r"\b(STATS_DATA|CONFIG_DATA|DUMP_ACK):\s*([0-9A-Fa-f]+)")
SENDER_LOG_RE = re.compile(r"\bsender:")

ROTATION_SOURCES = {
    0: "CSI",
    1: "ESPNOW",
    2: "CSI DR",
    3: "ESPNOW DR",
}
TRANSLATION_METHODS = {0: "square", 1: "sine", 2: "linear"}
LED_DISPLAY_MODES = {0: "simple angle", 1: "rpm display", 2: "picture", 3: "RSSI PoV"}

TRANSLATION_LENGTH_DEFAULT = 0.05
TRANSLATION_LENGTH_STEP = 0.05
HELD_DIRECTION_TIMEOUT_SEC = 0.15
BACKSPACE_KEYS = {8, 127}
DIRECTION_KEYS = {
    ord("w"): "w",
    ord("W"): "w",
    ord("a"): "a",
    ord("A"): "a",
    ord("s"): "s",
    ord("S"): "s",
    ord("d"): "d",
    ord("D"): "d",
}


@dataclass
class ControlState:
    throttle: int = 0
    vector_x: float = 0.0
    vector_y: float = 0.0


@dataclass
class ConfigState:
    dshot_pin_a: int = 13
    dshot_pin_b: int = 9
    led_pin: int = 12
    rotation_source: int = 1
    step_lag: int = 5
    step_window: int = 5
    throttle_multiplier: float = 2.0
    translation_multiplier: float = 4.0
    correlation_window: int = 1000
    smoothing_window: int = 20
    phase_offset: float = 0.0
    translation_method: int = 1
    led_display_mode: int = 3


@dataclass
class StatsState:
    rssi_mean: float = 0.0
    rssi_var: float = 0.0
    pkts_per_sec: int = 0
    last_rssi: int = 0
    rotation_rate: float = 0.0
    vector_x: float = 0.0
    vector_y: float = 0.0
    autocorrelation_time: int = 0
    updated_at: float = 0.0


@dataclass
class ConfigField:
    name: str
    attr: str
    step: float
    min_value: float
    max_value: float
    is_float: bool = False
    labels: Optional[dict[int, str]] = None


CONFIG_FIELDS = [
    ConfigField("dshot_pin_a", "dshot_pin_a", 1, 0, 48),
    ConfigField("dshot_pin_b", "dshot_pin_b", 1, 0, 48),
    ConfigField("led_pin", "led_pin", 1, 0, 48),
    ConfigField("rotation_source", "rotation_source", 1, 0, 3, labels=ROTATION_SOURCES),
    ConfigField("step_lag", "step_lag", 1, 0, 65535),
    ConfigField("step_window", "step_window", 1, 0, 65535),
    ConfigField("throttle_multiplier", "throttle_multiplier", 0.1, -100.0, 100.0, True),
    ConfigField("translation_multiplier", "translation_multiplier", 0.1, -100.0, 100.0, True),
    ConfigField("correlation_window", "correlation_window", 10, 0, 65535),
    ConfigField("smoothing_window", "smoothing_window", 1, 0, 65535),
    ConfigField("phase_offset", "phase_offset", 0.05, -math.pi * 4, math.pi * 4, True),
    ConfigField(
        "translation_method",
        "translation_method",
        1,
        0,
        2,
        labels=TRANSLATION_METHODS,
    ),
    ConfigField(
        "led_display_mode",
        "led_display_mode",
        1,
        0,
        3,
        labels=LED_DISPLAY_MODES,
    ),
]


class AppState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.control = ControlState()
        self.config_draft = ConfigState()
        self.config_state: Optional[ConfigState] = None
        self.config_dirty = False
        self.stats: Optional[StatsState] = None
        self.logs: Deque[str] = deque(maxlen=12)
        self.raw_frames: Deque[str] = deque(maxlen=8)
        self.translation_vector_length = TRANSLATION_LENGTH_DEFAULT
        self.direction_held_until = {"w": 0.0, "a": 0.0, "s": 0.0, "d": 0.0}
        self.running = True
        self.send_enabled = True
        self.connected = False
        self.last_error = ""
        self.last_ack = ""
        self.last_tx = 0.0
        self.last_rx = 0.0
        self.tx_control_count = 0
        self.tx_config_count = 0
        self.tx_dump_count = 0
        self.rx_stats_count = 0
        self.rx_config_count = 0
        self.rx_ack_count = 0
        self.rx_bad_count = 0
        self.selected_config = 0

    def log(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        with self.lock:
            self.logs.append(line[-240:])

    def set_error(self, text: str) -> None:
        with self.lock:
            self.last_error = text[-160:]


class SenderSerial:
    def __init__(self, port: str, baud: int, state: AppState) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run with the ESP-IDF Python env or install pyserial.")
        self.state = state
        self.serial = serial.Serial(port, baud, timeout=0.05, write_timeout=0.5)
        self.serial.dtr = False
        self.serial.rts = False
        self.write_lock = threading.Lock()
        self.reader = threading.Thread(target=self._read_loop, name="serial-reader", daemon=True)
        self.reader.start()
        with self.state.lock:
            self.state.connected = True

    def close(self) -> None:
        try:
            self.serial.close()
        finally:
            with self.state.lock:
                self.state.connected = False

    def write_packet(self, payload: bytes) -> None:
        checksum = xor_checksum(payload)
        frame = bytes([START_BYTE_TX]) + escape_tx_bytes(payload + bytes([checksum]))
        with self.write_lock:
            self.serial.write(frame)
            self.serial.flush()
        with self.state.lock:
            self.state.last_tx = time.time()

    def send_control(self, control: ControlState) -> None:
        payload = CONTROL_STRUCT.pack(
            APP_PACKET_TYPE_CONTROL,
            APP_PROTOCOL_MAGIC,
            clamp_int(control.throttle, 0, 65535),
            float(control.vector_x),
            float(control.vector_y),
        )
        self.write_packet(payload)
        with self.state.lock:
            self.state.tx_control_count += 1

    def send_config(self, config: ConfigState) -> None:
        payload = CONFIG_STRUCT.pack(
            APP_PACKET_TYPE_CONFIG_SET,
            APP_PROTOCOL_MAGIC,
            clamp_int(config.dshot_pin_a, 0, 255),
            clamp_int(config.dshot_pin_b, 0, 255),
            clamp_int(config.led_pin, 0, 255),
            clamp_int(config.rotation_source, 0, 255),
            clamp_int(config.step_lag, 0, 65535),
            clamp_int(config.step_window, 0, 65535),
            float(config.throttle_multiplier),
            float(config.translation_multiplier),
            clamp_int(config.correlation_window, 0, 65535),
            clamp_int(config.smoothing_window, 0, 65535),
            float(config.phase_offset),
            clamp_int(config.translation_method, 0, 255),
            clamp_int(config.led_display_mode, 0, 255),
        )
        self.write_packet(payload)
        with self.state.lock:
            self.state.tx_config_count += 1
            self.state.last_ack = "config sent; waiting for CONFIG_DATA"

    def send_dump(self) -> None:
        self.write_packet(bytes([APP_PACKET_TYPE_CMD_DUMP, APP_PROTOCOL_MAGIC]))
        with self.state.lock:
            self.state.tx_dump_count += 1

    def _read_loop(self) -> None:
        buf = bytearray()
        while self.state.running:
            try:
                chunk = self.serial.read(256)
            except Exception as exc:
                self.state.set_error(f"serial read failed: {exc}")
                break
            if not chunk:
                continue
            for byte in chunk:
                if byte in (10, 13):
                    if buf:
                        self._handle_line(bytes(buf).decode("utf-8", errors="replace"))
                        buf.clear()
                else:
                    buf.append(byte)
                    if len(buf) > 4096:
                        self._handle_line(bytes(buf).decode("utf-8", errors="replace"))
                        buf.clear()

    def _handle_line(self, line: str) -> None:
        clean = ANSI_RE.sub("", line).strip()
        if not clean:
            return
        match = FRAME_RE.search(clean)
        if match:
            label, hex_text = match.groups()
            parse_rx_frame(label, hex_text, self.state)
        else:
            self.state.log(clean)


def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def escape_tx_bytes(data: bytes) -> bytes:
    escaped = bytearray()
    for byte in data:
        if byte in ESCAPED_TX_BYTES:
            escaped.append(ESCAPE_BYTE_TX)
            escaped.append(byte ^ ESCAPE_XOR_TX)
        else:
            escaped.append(byte)
    return bytes(escaped)


def clamp_int(value: float, low: int, high: int) -> int:
    return max(low, min(high, int(round(value))))


def parse_rx_frame(label: str, hex_text: str, state: AppState) -> None:
    try:
        frame = bytes.fromhex(hex_text)
    except ValueError:
        state.set_error(f"bad {label} hex")
        with state.lock:
            state.rx_bad_count += 1
        return

    with state.lock:
        state.raw_frames.append(f"{label}: {hex_text[-120:]}")
        state.last_rx = time.time()

    if len(frame) < 3 or frame[0] != START_BYTE_RX:
        state.set_error(f"bad {label} frame")
        with state.lock:
            state.rx_bad_count += 1
        return

    payload = frame[1:-1]
    if xor_checksum(payload) != frame[-1]:
        state.set_error(f"{label} checksum failed")
        with state.lock:
            state.rx_bad_count += 1
        return
    if len(payload) < APP_CMD_DUMP_PACKET_SIZE or payload[1] != APP_PROTOCOL_MAGIC:
        state.set_error(f"{label} protocol magic failed")
        with state.lock:
            state.rx_bad_count += 1
        return

    packet_type = payload[0]
    try:
        if packet_type == APP_PACKET_TYPE_STATS and len(payload) == STATS_STRUCT.size:
            values = STATS_STRUCT.unpack(payload)
            stats = StatsState(
                rssi_mean=values[2],
                rssi_var=values[3],
                pkts_per_sec=values[4],
                last_rssi=unpack_int8(values[5]),
                rotation_rate=values[6],
                vector_x=values[7],
                vector_y=values[8],
                autocorrelation_time=values[9],
                updated_at=time.time(),
            )
            with state.lock:
                state.stats = stats
                state.rx_stats_count += 1
        elif packet_type == APP_PACKET_TYPE_CONFIG_STATE and len(payload) == CONFIG_STRUCT.size:
            config = unpack_config(payload)
            with state.lock:
                state.config_state = config
                if not state.config_dirty:
                    state.config_draft = copy_config(config)
                elif configs_match(state.config_draft, config):
                    state.config_dirty = False
                    state.last_ack = "config applied"
                else:
                    state.last_ack = "config state received"
                state.rx_config_count += 1
        elif packet_type == APP_PACKET_TYPE_CMD_ACK and len(payload) == APP_CMD_ACK_PACKET_SIZE:
            status = payload[2]
            with state.lock:
                state.last_ack = f"dump ack status={status}"
                state.rx_ack_count += 1
        else:
            state.set_error(f"unexpected {label} payload type=0x{packet_type:02X} len={len(payload)}")
            with state.lock:
                state.rx_bad_count += 1
    except struct.error as exc:
        state.set_error(f"{label} parse failed: {exc}")
        with state.lock:
            state.rx_bad_count += 1


def unpack_int8(value: int) -> int:
    return value - 256 if value > 127 else value


def unpack_config(payload: bytes) -> ConfigState:
    values = CONFIG_STRUCT.unpack(payload)
    return ConfigState(
        dshot_pin_a=values[2],
        dshot_pin_b=values[3],
        led_pin=values[4],
        rotation_source=values[5],
        step_lag=values[6],
        step_window=values[7],
        throttle_multiplier=values[8],
        translation_multiplier=values[9],
        correlation_window=values[10],
        smoothing_window=values[11],
        phase_offset=values[12],
        translation_method=values[13],
        led_display_mode=values[14],
    )


def format_field_value(config: ConfigState, field: ConfigField) -> str:
    value = getattr(config, field.attr)
    if field.labels is not None:
        return f"{value} ({field.labels.get(int(value), 'unknown')})"
    if field.is_float:
        return f"{float(value):.3f}"
    return str(value)


def adjust_config(config: ConfigState, field: ConfigField, direction: int) -> ConfigState:
    value = getattr(config, field.attr)
    if field.labels is not None:
        new_value = int(value) + direction
        if new_value > field.max_value:
            new_value = int(field.min_value)
        elif new_value < field.min_value:
            new_value = int(field.max_value)
    else:
        new_value = float(value) + direction * field.step
        new_value = max(field.min_value, min(field.max_value, new_value))
        if not field.is_float:
            new_value = int(round(new_value))
    return replace(config, **{field.attr: new_value})


def copy_config(config: ConfigState) -> ConfigState:
    return replace(config)


def configs_match(left: ConfigState, right: ConfigState) -> bool:
    for field in CONFIG_FIELDS:
        left_value = getattr(left, field.attr)
        right_value = getattr(right, field.attr)
        if field.is_float:
            if abs(float(left_value) - float(right_value)) > 1e-4:
                return False
        elif int(left_value) != int(right_value):
            return False
    return True


def compute_translation_vector(length: float, held: dict[str, bool]) -> tuple[float, float]:
    x_axis = int(held.get("d", False)) - int(held.get("a", False))
    y_axis = int(held.get("w", False)) - int(held.get("s", False))

    if x_axis == 0 and y_axis == 0:
        return 0.0, 0.0
    if x_axis != 0 and y_axis != 0:
        component = math.sqrt(max(0.0, length))
        return float(x_axis) * component, float(y_axis) * component
    return float(x_axis) * length, float(y_axis) * length


def refresh_translation_control(state: AppState, now: float) -> bool:
    with state.lock:
        held = {direction: held_until > now for direction, held_until in state.direction_held_until.items()}
        vector_x, vector_y = compute_translation_vector(state.translation_vector_length, held)
        control = state.control
        changed = not math.isclose(control.vector_x, vector_x, abs_tol=1e-9) or not math.isclose(
            control.vector_y,
            vector_y,
            abs_tol=1e-9,
        )
        if changed:
            state.control = replace(control, vector_x=vector_x, vector_y=vector_y)
        return changed


def clear_direction_holds(state: AppState) -> None:
    state.direction_held_until = {direction: 0.0 for direction in state.direction_held_until}


def find_default_port(baud: int) -> Optional[str]:
    env_port = os.environ.get("SENDER_PORT")
    if env_port:
        return env_port

    explicit_ports = [
        "/dev/ttyACM0",
        "/dev/ttyACM1",
    ]
    patterns = [
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/ttyUSB*",
    ]
    candidates: list[str] = []
    candidates.extend(port for port in explicit_ports if os.path.exists(port))
    for pattern in patterns:
        candidates.extend(sorted(glob.glob(pattern)))
    candidates.extend(sorted(glob.glob("/dev/ttyACM*")))

    candidates = list(dict.fromkeys(candidates))
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]

    for candidate in candidates:
        if candidate.endswith("usbmodem1101"):
            return candidate

    sender_port = probe_sender_ports(candidates, baud)
    if sender_port:
        return sender_port

    return candidates[-1]


def probe_sender_ports(candidates: list[str], baud: int) -> Optional[str]:
    if serial is None:
        return None

    for port in candidates:
        if probe_sender_port(port, baud, active=False):
            return port

    for port in candidates:
        if probe_sender_port(port, baud, active=True):
            return port

    return None


def probe_sender_port(port: str, baud: int, active: bool) -> bool:
    try:
        with serial.Serial(port, baud, timeout=0.05, write_timeout=0.05) as probe:
            probe.dtr = False
            probe.rts = False
            probe_frame = b""
            if active:
                payload = bytes([APP_PACKET_TYPE_CMD_DUMP, APP_PROTOCOL_MAGIC])
                probe_frame = bytes([START_BYTE_TX]) + escape_tx_bytes(
                    payload + bytes([xor_checksum(payload)])
                )
            deadline = time.time() + 1.2
            next_probe_at = 0.0
            buf = bytearray()
            while time.time() < deadline:
                now = time.time()
                if active and now >= next_probe_at:
                    probe.write(probe_frame)
                    probe.flush()
                    next_probe_at = now + 0.25
                chunk = probe.read(256)
                if not chunk:
                    continue
                for byte in chunk:
                    if byte in (10, 13):
                        if not buf:
                            continue
                        line = ANSI_RE.sub("", bytes(buf).decode("utf-8", errors="replace")).strip()
                        buf.clear()
                        if FRAME_RE.search(line) or SENDER_LOG_RE.search(line):
                            return True
                    else:
                        buf.append(byte)
                        if len(buf) > 4096:
                            buf.clear()
    except Exception:
        return False
    return False


def draw_line(stdscr: "curses._CursesWindow", y: int, x: int, text: str, attr: int = 0) -> None:
    height, width = stdscr.getmaxyx()
    if y < 0 or y >= height or x >= width:
        return
    text = text[: max(0, width - x - 1)]
    try:
        stdscr.addstr(y, x, text, attr)
    except curses.error:
        pass


def draw_config(
    stdscr: "curses._CursesWindow",
    y: int,
    x: int,
    title: str,
    config: Optional[ConfigState],
    selected: Optional[int],
) -> int:
    draw_line(stdscr, y, x, title, curses.A_BOLD)
    y += 1
    if config is None:
        draw_line(stdscr, y, x, "no config state received")
        return y + 1
    for idx, field in enumerate(CONFIG_FIELDS):
        marker = ">" if selected == idx else " "
        attr = curses.A_REVERSE if selected == idx else 0
        draw_line(
            stdscr,
            y,
            x,
            f"{marker} {field.name:<23} {format_field_value(config, field):>16}",
            attr,
        )
        y += 1
    return y


def draw_config_table(
    stdscr: "curses._CursesWindow",
    y: int,
    x: int,
    draft: ConfigState,
    receiver: Optional[ConfigState],
    selected: int,
    dirty: bool,
) -> int:
    title = "Config *draft edited" if dirty else "Config"
    draw_line(stdscr, y, x, title, curses.A_BOLD)
    draw_line(stdscr, y, x + 25, "draft", curses.A_BOLD)
    draw_line(stdscr, y, x + 45, "receiver", curses.A_BOLD)
    y += 1
    for idx, field in enumerate(CONFIG_FIELDS):
        marker = ">" if selected == idx else " "
        attr = curses.A_REVERSE if selected == idx else 0
        draft_text = format_field_value(draft, field)
        rx_text = format_field_value(receiver, field) if receiver is not None else "-"
        draw_line(
            stdscr,
            y,
            x,
            f"{marker} {field.name:<22} {draft_text:<18} {rx_text:<18}",
            attr,
        )
        y += 1
    return y


def snapshot(state: AppState) -> dict:
    with state.lock:
        now = time.time()
        return {
            "control": replace(state.control),
            "config_draft": replace(state.config_draft),
            "config_state": replace(state.config_state) if state.config_state else None,
            "config_dirty": state.config_dirty,
            "stats": replace(state.stats) if state.stats else None,
            "logs": list(state.logs),
            "raw_frames": list(state.raw_frames),
            "translation_vector_length": state.translation_vector_length,
            "held_directions": [
                direction
                for direction in ("w", "a", "s", "d")
                if state.direction_held_until.get(direction, 0.0) > now
            ],
            "running": state.running,
            "send_enabled": state.send_enabled,
            "connected": state.connected,
            "last_error": state.last_error,
            "last_ack": state.last_ack,
            "last_tx": state.last_tx,
            "last_rx": state.last_rx,
            "tx_control_count": state.tx_control_count,
            "tx_config_count": state.tx_config_count,
            "tx_dump_count": state.tx_dump_count,
            "rx_stats_count": state.rx_stats_count,
            "rx_config_count": state.rx_config_count,
            "rx_ack_count": state.rx_ack_count,
            "rx_bad_count": state.rx_bad_count,
            "selected_config": state.selected_config,
        }


def run_ui(stdscr: "curses._CursesWindow", link: SenderSerial, state: AppState, send_rate_hz: float) -> None:
    curses.curs_set(0)
    stdscr.keypad(True)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    last_send = 0.0
    send_interval = 1.0 / max(send_rate_hz, 1.0)

    while state.running:
        key = stdscr.getch()
        now = time.time()
        control_send_requested = handle_key(key, link, state, now)
        translation_changed = refresh_translation_control(state, time.time())
        data = snapshot(state)
        now = time.time()
        if control_send_requested or translation_changed or (data["send_enabled"] and now - last_send >= send_interval):
            link.send_control(data["control"])
            last_send = now

        render(stdscr, data)

    zero = ControlState()
    link.send_control(zero)


def handle_key(key: int, link: SenderSerial, state: AppState, now: float) -> bool:
    if key == -1:
        return False

    if key == curses.KEY_BACKSPACE:
        key = 8

    with state.lock:
        control = state.control
        config = state.config_draft
        selected = state.selected_config

        if key in (27, 3):
            state.control = ControlState()
            clear_direction_holds(state)
            state.running = False
            return True
        if key in (ord("p"), ord("P")):
            state.send_enabled = not state.send_enabled
            return False
        if key in (ord("x"), ord("X")):
            state.control = ControlState()
            clear_direction_holds(state)
            return True
        if ord("1") <= key <= ord("9"):
            state.control = replace(control, throttle=(key - ord("0")) * 100)
            return True
        if key in (ord("+"), ord("=")):
            state.control = replace(control, throttle=clamp_int(control.throttle + 50, 0, 65535))
            return True
        if key in (ord("-"), ord("_")):
            state.control = replace(control, throttle=clamp_int(control.throttle - 50, 0, 65535))
            return True
        if key in BACKSPACE_KEYS:
            state.control = replace(control, throttle=0)
            return True
        if key in (ord("q"), ord("Q")):
            state.translation_vector_length = max(
                0.0,
                round(state.translation_vector_length - TRANSLATION_LENGTH_STEP, 10),
            )
            return True
        if key in (ord("e"), ord("E")):
            state.translation_vector_length = round(state.translation_vector_length + TRANSLATION_LENGTH_STEP, 10)
            return True
        if key in DIRECTION_KEYS:
            state.direction_held_until[DIRECTION_KEYS[key]] = now + HELD_DIRECTION_TIMEOUT_SEC
            return True
        if key in (ord("z"), ord("Z")):
            clear_direction_holds(state)
            state.control = replace(control, vector_x=0.0, vector_y=0.0)
            return True
        elif key in (ord("["), curses.KEY_LEFT):
            state.selected_config = (selected - 1) % len(CONFIG_FIELDS)
        elif key in (ord("]"), curses.KEY_RIGHT):
            state.selected_config = (selected + 1) % len(CONFIG_FIELDS)
        elif key in (ord(","), ord("<")):
            state.config_draft = adjust_config(config, CONFIG_FIELDS[selected], -1)
            state.config_dirty = True
        elif key in (ord("."), ord(">")):
            state.config_draft = adjust_config(config, CONFIG_FIELDS[selected], 1)
            state.config_dirty = True
        elif key in (ord("r"), ord("R")):
            if state.config_state is not None:
                state.config_draft = copy_config(state.config_state)
                state.config_dirty = False
        elif key in (10, 13, curses.KEY_ENTER):
            config_to_send = copy_config(state.config_draft)
            send_kind = "config"
        elif key in (ord("g"), ord("G")):
            send_kind = "dump"
        else:
            send_kind = ""

    if "send_kind" in locals() and send_kind == "config":
        link.send_config(config_to_send)
    elif "send_kind" in locals() and send_kind == "dump":
        link.send_dump()
    return False


def render(stdscr: "curses._CursesWindow", data: dict) -> None:
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    now = time.time()

    control: ControlState = data["control"]
    stats: Optional[StatsState] = data["stats"]

    draw_line(
        stdscr,
        0,
        0,
        "Sender TUI  Esc quit | x stop | p pause | 1-9 throttle | +/- throttle | Backspace zero | WASD direction | Q/E length | [] config | ,/. edit | enter send | r sync | g dump",
        curses.A_BOLD,
    )
    draw_line(stdscr, 1, 0, "-" * max(0, width - 1))

    status = "connected" if data["connected"] else "disconnected"
    stream = "sending" if data["send_enabled"] else "paused"
    last_tx = age_text(now, data["last_tx"])
    last_rx = age_text(now, data["last_rx"])
    draw_line(
        stdscr,
        2,
        0,
        f"Status: {status}, control stream {stream}, last tx {last_tx}, last rx {last_rx}",
    )
    if data["last_error"]:
        draw_line(stdscr, 3, 0, f"Last error: {data['last_error']}", curses.A_BOLD)
    elif data["last_ack"]:
        draw_line(stdscr, 3, 0, f"Last ack: {data['last_ack']}")

    right_x = 44 if width >= 88 else 0
    draw_line(stdscr, 5, 0, "Host -> Sender -> Receiver", curses.A_BOLD)
    held = "".join(data["held_directions"]) or "-"
    draw_line(
        stdscr,
        6,
        0,
        f"throttle {control.throttle:>6}  vector {control.vector_x:>6.2f}, {control.vector_y:>6.2f}",
    )
    draw_line(stdscr, 7, 0, f"length {data['translation_vector_length']:>5.2f}  dir {held}")
    draw_line(stdscr, 8, 0, f"tx ctl={data['tx_control_count']} cfg={data['tx_config_count']} dump={data['tx_dump_count']}")

    stats_x = right_x
    stats_y = 5 if right_x else 10
    draw_line(stdscr, stats_y, stats_x, "Receiver -> Sender -> Host", curses.A_BOLD)
    if stats is None:
        draw_line(stdscr, stats_y + 1, stats_x, "no stats received")
    else:
        draw_line(stdscr, stats_y + 1, stats_x, f"age {age_text(now, stats.updated_at):>6} pkts {stats.pkts_per_sec:>6} rssi {stats.last_rssi:>4}")
        draw_line(stdscr, stats_y + 2, stats_x, f"mean {stats.rssi_mean:>7.2f} var {stats.rssi_var:>7.2f}")
        draw_line(stdscr, stats_y + 3, stats_x, f"rot {stats.rotation_rate:>7.3f} vec {stats.vector_x:>5.2f}, {stats.vector_y:>5.2f}")
        draw_line(stdscr, stats_y + 4, stats_x, f"autocorr_us {stats.autocorrelation_time:>8}")
    draw_line(stdscr, stats_y + 5, stats_x, f"rx stats={data['rx_stats_count']} cfg={data['rx_config_count']} ack={data['rx_ack_count']} bad={data['rx_bad_count']}")

    config_y = 11 if right_x else 17
    config_end_y = draw_config_table(
        stdscr,
        config_y,
        0,
        data["config_draft"],
        data["config_state"],
        data["selected_config"],
        data["config_dirty"],
    )

    log_y = max(config_end_y + 1, height - 8)
    if log_y >= height - 2:
        stdscr.refresh()
        return
    draw_line(stdscr, log_y, 0, "-" * max(0, width - 1))
    draw_line(stdscr, log_y + 1, 0, "Device Logs", curses.A_BOLD)
    y = log_y + 2
    for line in data["logs"][-5:]:
        draw_line(stdscr, y, 0, line)
        y += 1

    if width >= 110:
        raw_x = 56
        draw_line(stdscr, log_y + 1, raw_x, "Recent Parsed Frames", curses.A_BOLD)
        y = log_y + 2
        for line in data["raw_frames"][-5:]:
            draw_line(stdscr, y, raw_x, line)
            y += 1

    stdscr.refresh()


def age_text(now: float, timestamp: float) -> str:
    if not timestamp:
        return "never"
    age = max(0.0, now - timestamp)
    if age < 1.0:
        return f"{age * 1000:.0f}ms"
    return f"{age:.1f}s"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="TUI for the ESP32 Sender serial protocol")
    parser.add_argument(
        "-p",
        "--port",
        default=None,
        help="serial port; default uses SENDER_PORT or auto-detects the sender",
    )
    parser.add_argument("-b", "--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument("--send-rate", type=float, default=30.0, help="control packets per second while unpaused")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    port = args.port or find_default_port(args.baud)
    if port is None:
        print("No serial port found. Pass --port /dev/cu.usbmodemXXXX.", file=sys.stderr)
        return 2
    state = AppState()
    try:
        link = SenderSerial(port, args.baud, state)
    except Exception as exc:
        print(f"Failed to open {port}: {exc}", file=sys.stderr)
        return 1

    try:
        curses.wrapper(run_ui, link, state, args.send_rate)
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        try:
            link.send_control(ControlState())
        except Exception:
            pass
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

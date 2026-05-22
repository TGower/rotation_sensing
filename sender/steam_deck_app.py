#!/usr/bin/env python3
"""Steam Deck controller UI for the Sender serial protocol."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

try:
    import pygame
except ImportError:  # pragma: no cover - exercised only on unprepared hosts.
    pygame = None

from tui import (
    CONFIG_FIELDS,
    AppState,
    ConfigState,
    ControlState,
    SenderSerial,
    StatsState,
    adjust_config,
    age_text,
    clamp_int,
    copy_config,
    find_default_port,
    format_field_value,
    snapshot,
)


SCREEN_SIZE = (1280, 800)
FPS = 60

BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LEFT_BUMPER = 4
BUTTON_RIGHT_BUMPER = 5
BUTTON_BACK = 6
BUTTON_START = 7
BUTTON_DPAD_UP = 11
BUTTON_DPAD_DOWN = 12
BUTTON_DPAD_LEFT = 13
BUTTON_DPAD_RIGHT = 14
BUMPER_SHUTDOWN_SECONDS = 1.0
MODE_SETTINGS = "settings"
MODE_CONTROL = "control"
SETTINGS_EXIT_SELECTION = -1
LOCAL_THROTTLE_SCALAR_MIN = 1.0
LOCAL_THROTTLE_SCALAR_MAX = 2.0
LOCAL_THROTTLE_SCALAR_RAMP_SECONDS = 1.0

COLOR_BG = (9, 12, 16)
COLOR_PANEL = (20, 26, 34)
COLOR_PANEL_ALT = (25, 33, 43)
COLOR_SELECTED = (48, 103, 160)
COLOR_BORDER = (64, 78, 94)
COLOR_TEXT = (230, 235, 241)
COLOR_MUTED = (145, 157, 170)
COLOR_GREEN = (83, 220, 135)
COLOR_RED = (246, 84, 84)
COLOR_YELLOW = (235, 192, 88)
COLOR_BLUE = (95, 170, 245)
COLOR_CYAN = (65, 220, 230)


class DeckInput:
    def __init__(
        self,
        joystick: Optional["pygame.joystick.Joystick"],
        lx_axis: int,
        ly_axis: int,
        rt_axis: int,
        deadzone: float,
        max_throttle: int,
        invert_y: bool,
    ) -> None:
        self.joystick = joystick
        self.lx_axis = lx_axis
        self.ly_axis = ly_axis
        self.rt_axis = rt_axis
        self.deadzone = deadzone
        self.max_throttle = max_throttle
        self.invert_y = invert_y
        self.stop_held = False
        self.keyboard_boost_held = False
        self.bumper_shutdown_started_at: Optional[float] = None

    def axis(self, index: int, default: float = 0.0) -> float:
        if self.joystick is None or index < 0 or index >= self.joystick.get_numaxes():
            return default
        return float(self.joystick.get_axis(index))

    def button(self, index: int) -> bool:
        if self.joystick is None or index < 0 or index >= self.joystick.get_numbuttons():
            return False
        return bool(self.joystick.get_button(index))

    def shutdown_requested(self, now: float) -> bool:
        if self.button(BUTTON_LEFT_BUMPER) and self.button(BUTTON_RIGHT_BUMPER):
            if self.bumper_shutdown_started_at is None:
                self.bumper_shutdown_started_at = now
            return now - self.bumper_shutdown_started_at >= BUMPER_SHUTDOWN_SECONDS

        self.bumper_shutdown_started_at = None
        return False

    def stick_value(self, index: int) -> float:
        value = self.axis(index)
        if abs(value) < self.deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    def trigger_value(self) -> float:
        raw = self.axis(self.rt_axis, -1.0)
        if raw < -0.05:
            value = (raw + 1.0) * 0.5
        else:
            value = raw
        if value < self.deadzone:
            return 0.0
        return min(1.0, max(0.0, value))

    def boost_held(self) -> bool:
        return self.keyboard_boost_held or self.button(BUTTON_A)

    def control(self, throttle_scalar: float = 1.0) -> ControlState:
        if self.stop_held:
            return ControlState()

        x = self.stick_value(self.lx_axis)
        y = self.stick_value(self.ly_axis)
        if self.invert_y:
            y = -y

        throttle = clamp_int(
            self.trigger_value() * self.max_throttle * throttle_scalar,
            0,
            65535,
        )
        return ControlState(
            throttle=throttle,
            vector_x=float(x),
            vector_y=float(y),
        )


class Fonts:
    def __init__(self) -> None:
        self.title = pygame.font.SysFont("DejaVu Sans", 34, bold=True)
        self.big = pygame.font.SysFont("DejaVu Sans", 46, bold=True)
        self.med = pygame.font.SysFont("DejaVu Sans", 24, bold=True)
        self.body = pygame.font.SysFont("DejaVu Sans", 19)
        self.small = pygame.font.SysFont("DejaVu Sans", 15)
        self.mono = pygame.font.SysFont("DejaVu Sans Mono", 16)


def draw_text(
    surface: "pygame.Surface",
    font: "pygame.font.Font",
    text: str,
    pos: tuple[int, int],
    color: tuple[int, int, int] = COLOR_TEXT,
) -> None:
    surface.blit(font.render(text, True, color), pos)


def draw_panel(
    surface: "pygame.Surface",
    rect: "pygame.Rect",
    title: Optional[str] = None,
    font: Optional["pygame.font.Font"] = None,
) -> None:
    pygame.draw.rect(surface, COLOR_PANEL, rect, border_radius=8)
    pygame.draw.rect(surface, COLOR_BORDER, rect, width=1, border_radius=8)
    if title and font is not None:
        draw_text(surface, font, title, (rect.x + 16, rect.y + 12), COLOR_TEXT)


def stat_age_color(now: float, stats: Optional[StatsState]) -> tuple[int, int, int]:
    if stats is None:
        return COLOR_RED
    age = now - stats.updated_at
    if age < 0.5:
        return COLOR_GREEN
    if age < 2.0:
        return COLOR_YELLOW
    return COLOR_RED


def fmt_float(value: float, digits: int = 2) -> str:
    if math.isnan(value) or math.isinf(value):
        return "-"
    return f"{value:.{digits}f}"


def config_value(config: Optional[ConfigState], idx: int) -> str:
    if config is None:
        return "-"
    return format_field_value(config, CONFIG_FIELDS[idx])


def mutate_config_selection(state, delta: int) -> None:
    with state.lock:
        selection_count = len(CONFIG_FIELDS) + 1
        current = state.selected_config + 1
        state.selected_config = ((current + delta) % selection_count) - 1


def adjust_selected_config(state, direction: int) -> None:
    with state.lock:
        selected = state.selected_config
        if selected == SETTINGS_EXIT_SELECTION:
            return
        state.config_draft = adjust_config(
            state.config_draft, CONFIG_FIELDS[selected], direction
        )
        state.config_dirty = True


def copy_receiver_config_to_draft(state) -> None:
    with state.lock:
        if state.config_state is not None:
            state.config_draft = copy_config(state.config_state)
            state.config_dirty = False
            state.last_ack = "draft synced from receiver"


def toggle_mode(link: SenderSerial, state, mode: str) -> str:
    next_mode = MODE_CONTROL if mode == MODE_SETTINGS else MODE_SETTINGS
    with state.lock:
        state.last_ack = f"{next_mode} mode"

    if next_mode == MODE_SETTINGS:
        zero = ControlState()
        with state.lock:
            state.control = zero
        try:
            link.send_control(zero)
        except Exception as exc:
            state.set_error(f"settings mode zero send failed: {exc}")

    return next_mode


def toggle_sending(state) -> None:
    with state.lock:
        state.send_enabled = not state.send_enabled


def set_zero_control(state) -> None:
    with state.lock:
        state.control = ControlState()


def exit_application(state) -> None:
    with state.lock:
        state.running = False


def settings_exit_selected(state) -> bool:
    with state.lock:
        return state.selected_config == SETTINGS_EXIT_SELECTION


def send_config(link: SenderSerial, state) -> None:
    with state.lock:
        config = copy_config(state.config_draft)
    try:
        link.send_config(config)
    except Exception as exc:
        state.set_error(f"config send failed: {exc}")


def update_local_throttle_scalar(current: float, boost_held: bool, dt: float) -> float:
    if dt <= 0.0:
        return max(LOCAL_THROTTLE_SCALAR_MIN, min(LOCAL_THROTTLE_SCALAR_MAX, current))

    step = dt / LOCAL_THROTTLE_SCALAR_RAMP_SECONDS
    if boost_held:
        return min(LOCAL_THROTTLE_SCALAR_MAX, current + step)
    return max(LOCAL_THROTTLE_SCALAR_MIN, current - step)


def update_control_from_deck(
    state,
    deck: DeckInput,
    mode: str,
    local_throttle_scalar: float,
) -> bool:
    with state.lock:
        current = state.control
    if mode == MODE_SETTINGS:
        next_control = ControlState()
    else:
        next_control = deck.control(local_throttle_scalar)
    changed = (
        current.throttle != next_control.throttle
        or not math.isclose(current.vector_x, next_control.vector_x, abs_tol=1e-4)
        or not math.isclose(current.vector_y, next_control.vector_y, abs_tol=1e-4)
    )
    if changed:
        with state.lock:
            state.control = next_control
    return changed


def handle_button_down(button: int, link: SenderSerial, state, deck: DeckInput, mode: str) -> str:
    if button == BUTTON_A:
        if mode == MODE_SETTINGS:
            if settings_exit_selected(state):
                exit_application(state)
            else:
                send_config(link, state)
    elif button == BUTTON_B:
        if mode == MODE_SETTINGS:
            copy_receiver_config_to_draft(state)
    elif button == BUTTON_X:
        deck.stop_held = True
        set_zero_control(state)
        try:
            link.send_control(ControlState())
        except Exception as exc:
            state.set_error(f"stop send failed: {exc}")
    elif button == BUTTON_Y:
        mode = toggle_mode(link, state, mode)
    elif button == BUTTON_START:
        toggle_sending(state)
    elif button == BUTTON_BACK:
        exit_application(state)
    elif mode != MODE_SETTINGS:
        pass
    elif button == BUTTON_DPAD_UP:
        mutate_config_selection(state, -1)
    elif button == BUTTON_DPAD_DOWN:
        mutate_config_selection(state, 1)
    elif button == BUTTON_DPAD_LEFT:
        adjust_selected_config(state, -1)
    elif button == BUTTON_DPAD_RIGHT:
        adjust_selected_config(state, 1)
    return mode


def handle_button_up(button: int, deck: DeckInput) -> None:
    if button == BUTTON_X:
        deck.stop_held = False


def handle_key_down(key: int, link: SenderSerial, state, deck: DeckInput, mode: str) -> str:
    if key in (pygame.K_ESCAPE, pygame.K_q):
        with state.lock:
            state.running = False
    elif key == pygame.K_y:
        mode = toggle_mode(link, state, mode)
    elif key == pygame.K_SPACE:
        deck.keyboard_boost_held = True
        if mode == MODE_SETTINGS:
            if settings_exit_selected(state):
                exit_application(state)
            else:
                send_config(link, state)
    elif key in (pygame.K_UP, pygame.K_w) and mode == MODE_SETTINGS:
        mutate_config_selection(state, -1)
    elif key in (pygame.K_DOWN, pygame.K_s) and mode == MODE_SETTINGS:
        mutate_config_selection(state, 1)
    elif key in (pygame.K_LEFT, pygame.K_a) and mode == MODE_SETTINGS:
        adjust_selected_config(state, -1)
    elif key in (pygame.K_RIGHT, pygame.K_d) and mode == MODE_SETTINGS:
        adjust_selected_config(state, 1)
    elif key == pygame.K_RETURN and mode == MODE_SETTINGS:
        if settings_exit_selected(state):
            exit_application(state)
        else:
            send_config(link, state)
    elif key == pygame.K_r and mode == MODE_SETTINGS:
        copy_receiver_config_to_draft(state)
    elif key == pygame.K_g:
        try:
            link.send_dump()
        except Exception as exc:
            state.set_error(f"dump send failed: {exc}")
    elif key == pygame.K_p:
        toggle_sending(state)
    elif key == pygame.K_x:
        deck.stop_held = True
        set_zero_control(state)
        try:
            link.send_control(ControlState())
        except Exception as exc:
            state.set_error(f"stop send failed: {exc}")
    return mode


def handle_key_up(key: int, deck: DeckInput) -> None:
    if key == pygame.K_x:
        deck.stop_held = False
    elif key == pygame.K_SPACE:
        deck.keyboard_boost_held = False


def handle_hat(value: tuple[int, int], state, mode: str) -> None:
    if mode != MODE_SETTINGS:
        return

    x, y = value
    if y > 0:
        mutate_config_selection(state, -1)
    elif y < 0:
        mutate_config_selection(state, 1)
    if x < 0:
        adjust_selected_config(state, -1)
    elif x > 0:
        adjust_selected_config(state, 1)


def draw_stats(surface: "pygame.Surface", fonts: Fonts, data: dict, rect: "pygame.Rect") -> None:
    draw_panel(surface, rect)
    stats: Optional[StatsState] = data["stats"]
    now = time.time()

    draw_text(surface, fonts.med, "Receiver Stats", (rect.x + 18, rect.y + 14))
    age_color = stat_age_color(now, stats)
    draw_text(
        surface,
        fonts.body,
        f"age {age_text(now, stats.updated_at) if stats else 'never'}",
        (rect.right - 128, rect.y + 18),
        age_color,
    )

    if stats is None:
        draw_text(surface, fonts.big, "No stats", (rect.x + 24, rect.y + 70), COLOR_RED)
        draw_text(surface, fonts.body, "Waiting for receiver frames", (rect.x + 28, rect.y + 130), COLOR_MUTED)
        return

    rpm = stats.rotation_rate * 60.0
    stat_blocks = [
        ("RPM", f"{rpm:,.0f}", COLOR_GREEN),
        ("Hz", fmt_float(stats.rotation_rate, 2), COLOR_CYAN),
        ("RSSI", f"{stats.last_rssi:d}", COLOR_YELLOW),
        ("Packets/s", f"{stats.pkts_per_sec:d}", COLOR_BLUE),
    ]

    block_w = (rect.width - 48) // 4
    y = rect.y + 58
    for idx, (label, value, color) in enumerate(stat_blocks):
        x = rect.x + 18 + idx * block_w
        draw_text(surface, fonts.small, label, (x, y), COLOR_MUTED)
        draw_text(surface, fonts.big, value, (x, y + 18), color)

    bottom_y = rect.y + 138
    draw_text(
        surface,
        fonts.body,
        f"RSSI mean {stats.rssi_mean:.2f}   var {stats.rssi_var:.2f}",
        (rect.x + 22, bottom_y),
        COLOR_TEXT,
    )
    draw_text(
        surface,
        fonts.body,
        f"receiver vector {stats.vector_x:.2f}, {stats.vector_y:.2f}",
        (rect.x + 22, bottom_y + 28),
        COLOR_TEXT,
    )
    draw_text(
        surface,
        fonts.body,
        f"autocorr {stats.autocorrelation_time / 1000.0:.2f} ms",
        (rect.x + 22, bottom_y + 56),
        COLOR_TEXT,
    )


def draw_control(
    surface: "pygame.Surface",
    fonts: Fonts,
    data: dict,
    rect: "pygame.Rect",
    controller_name: str,
) -> None:
    draw_panel(surface, rect, "Deck Control", fonts.med)
    control: ControlState = data["control"]
    mode = data["mode"]
    send_color = COLOR_GREEN if data["send_enabled"] else COLOR_YELLOW
    draw_text(
        surface,
        fonts.body,
        "sending" if data["send_enabled"] else "paused",
        (rect.right - 110, rect.y + 17),
        send_color,
    )

    gauge = pygame.Rect(rect.x + 24, rect.y + 66, 52, rect.height - 116)
    pygame.draw.rect(surface, (12, 16, 22), gauge, border_radius=5)
    pygame.draw.rect(surface, COLOR_BORDER, gauge, width=1, border_radius=5)
    fill_h = int(gauge.height * min(1.0, control.throttle / max(1, data["max_throttle"])))
    fill = pygame.Rect(gauge.x, gauge.bottom - fill_h, gauge.width, fill_h)
    pygame.draw.rect(surface, COLOR_GREEN, fill, border_radius=5)
    draw_text(surface, fonts.small, "RT", (gauge.x + 18, gauge.y - 24), COLOR_MUTED)
    draw_text(surface, fonts.med, f"{control.throttle}", (gauge.x - 4, gauge.bottom + 12), COLOR_TEXT)

    center = (rect.x + 230, rect.y + 144)
    radius = 82
    pygame.draw.circle(surface, (12, 16, 22), center, radius)
    pygame.draw.circle(surface, COLOR_BORDER, center, radius, width=2)
    pygame.draw.line(surface, COLOR_BORDER, (center[0] - radius, center[1]), (center[0] + radius, center[1]), 1)
    pygame.draw.line(surface, COLOR_BORDER, (center[0], center[1] - radius), (center[0], center[1] + radius), 1)

    dot_x = center[0] + int(max(-1.0, min(1.0, control.vector_x)) * radius)
    dot_y = center[1] - int(max(-1.0, min(1.0, control.vector_y)) * radius)
    pygame.draw.circle(surface, COLOR_CYAN, (dot_x, dot_y), 12)
    pygame.draw.line(surface, COLOR_CYAN, center, (dot_x, dot_y), 3)

    if mode == MODE_CONTROL:
        draw_text(surface, fonts.body, "Control Mode", (rect.x + 124, rect.y + 216), COLOR_GREEN)
        draw_text(
            surface,
            fonts.body,
            f"local throttle x{data['local_throttle_scalar']:.2f}",
            (rect.x + 124, rect.y + 244),
            COLOR_TEXT,
        )
    else:
        draw_text(surface, fonts.body, "Settings Mode", (rect.x + 124, rect.y + 216), COLOR_YELLOW)
        draw_text(surface, fonts.body, "controls forced zero", (rect.x + 124, rect.y + 244), COLOR_MUTED)
    draw_text(surface, fonts.body, f"vector {control.vector_x:+.2f}, {control.vector_y:+.2f}", (rect.x + 124, rect.y + 272))
    draw_text(surface, fonts.small, controller_name or "No controller detected", (rect.x + 22, rect.bottom - 48), COLOR_MUTED)


def draw_config(surface: "pygame.Surface", fonts: Fonts, data: dict, rect: "pygame.Rect") -> None:
    draw_panel(surface, rect)
    settings_enabled = data["mode"] == MODE_SETTINGS
    if not settings_enabled:
        draw_text(surface, fonts.med, "Control Mode", (rect.x + 16, rect.y + 12))
        draw_text(surface, fonts.body, "Settings locked while controlling", (rect.x + 18, rect.y + 58), COLOR_MUTED)
        draw_text(surface, fonts.small, "A throttle boost   Y settings", (rect.x + 18, rect.bottom - 32), COLOR_MUTED)
        return

    title = "Settings *" if data["config_dirty"] else "Settings"
    draw_text(surface, fonts.med, title, (rect.x + 16, rect.y + 12))
    draw_text(surface, fonts.small, "draft", (rect.x + 334, rect.y + 18), COLOR_MUTED)
    draw_text(surface, fonts.small, "receiver", (rect.x + 474, rect.y + 18), COLOR_MUTED)

    selected = data["selected_config"]
    exit_row = pygame.Rect(rect.x + 10, rect.y + 50, rect.width - 20, 32)
    if selected == SETTINGS_EXIT_SELECTION:
        pygame.draw.rect(surface, COLOR_SELECTED, exit_row, border_radius=5)
        exit_color = COLOR_TEXT
    else:
        pygame.draw.rect(surface, COLOR_PANEL_ALT, exit_row, border_radius=5)
        exit_color = COLOR_MUTED
    draw_text(surface, fonts.body, "Exit Application", (exit_row.x + 12, exit_row.y + 6), exit_color)
    draw_text(surface, fonts.small, "A", (exit_row.right - 28, exit_row.y + 9), exit_color)

    row_h = 26
    y = rect.y + 92
    for idx, field in enumerate(CONFIG_FIELDS):
        row = pygame.Rect(rect.x + 10, y - 3, rect.width - 20, row_h)
        if idx == selected and settings_enabled:
            pygame.draw.rect(surface, COLOR_SELECTED, row, border_radius=5)
        elif idx % 2:
            pygame.draw.rect(surface, COLOR_PANEL_ALT, row, border_radius=5)

        color = COLOR_TEXT if idx == selected and settings_enabled else COLOR_MUTED
        draft_color = COLOR_TEXT if settings_enabled else COLOR_MUTED
        draw_text(surface, fonts.mono, field.name[:22], (rect.x + 18, y), color)
        draw_text(surface, fonts.mono, config_value(data["config_draft"], idx)[:14], (rect.x + 332, y), draft_color)
        draw_text(surface, fonts.mono, config_value(data["config_state"], idx)[:14], (rect.x + 474, y), COLOR_TEXT)
        y += row_h

    footer_y = rect.bottom - 32
    footer = "D-PAD select/edit   A apply/exit   B sync   Y control"
    draw_text(surface, fonts.small, footer, (rect.x + 18, footer_y), COLOR_MUTED)


def draw_status(surface: "pygame.Surface", fonts: Fonts, data: dict, rect: "pygame.Rect") -> None:
    draw_panel(surface, rect)
    status = "connected" if data["connected"] else "disconnected"
    status_color = COLOR_GREEN if data["connected"] else COLOR_RED
    mode = "settings mode" if data["mode"] == MODE_SETTINGS else "control mode"
    mode_color = COLOR_YELLOW if data["mode"] == MODE_SETTINGS else COLOR_GREEN
    draw_text(surface, fonts.title, "Rotation Sender", (rect.x + 18, rect.y + 12))
    draw_text(surface, fonts.body, mode, (rect.x + 270, rect.y + 25), mode_color)
    draw_text(surface, fonts.body, status, (rect.x + 410, rect.y + 25), status_color)
    draw_text(surface, fonts.body, f"last tx {age_text(time.time(), data['last_tx'])}", (rect.x + 550, rect.y + 25), COLOR_MUTED)
    draw_text(surface, fonts.body, f"last rx {age_text(time.time(), data['last_rx'])}", (rect.x + 700, rect.y + 25), COLOR_MUTED)
    counters = (
        f"tx ctl {data['tx_control_count']} cfg {data['tx_config_count']} dump {data['tx_dump_count']}   "
        f"rx stats {data['rx_stats_count']} cfg {data['rx_config_count']} ack {data['rx_ack_count']} bad {data['rx_bad_count']}"
    )
    draw_text(surface, fonts.small, counters, (rect.x + 18, rect.y + 62), COLOR_MUTED)

    if data["last_error"]:
        draw_text(surface, fonts.body, data["last_error"], (rect.x + 860, rect.y + 25), COLOR_RED)
    elif data["last_ack"]:
        draw_text(surface, fonts.body, data["last_ack"], (rect.x + 860, rect.y + 25), COLOR_YELLOW)


def draw_logs(surface: "pygame.Surface", fonts: Fonts, data: dict, rect: "pygame.Rect") -> None:
    draw_panel(surface, rect, "Device Logs", fonts.med)
    y = rect.y + 48
    lines = data["logs"][-4:]
    if not lines:
        draw_text(surface, fonts.body, "No log lines", (rect.x + 18, y), COLOR_MUTED)
        return
    for line in lines:
        draw_text(surface, fonts.small, line[-150:], (rect.x + 18, y), COLOR_MUTED)
        y += 24


def render(
    screen: "pygame.Surface",
    fonts: Fonts,
    data: dict,
    controller_name: str,
) -> None:
    screen.fill(COLOR_BG)
    width, height = screen.get_size()
    margin = 16
    gap = 12
    header = pygame.Rect(margin, margin, width - margin * 2, 78)
    stats = pygame.Rect(margin, header.bottom + gap, width - margin * 2, 190)
    body_y = stats.bottom + gap
    body_h = min(300, max(260, height - body_y - margin - 172))
    control = pygame.Rect(margin, body_y, 520, body_h)
    config = pygame.Rect(
        control.right + gap,
        body_y,
        width - control.right - margin - gap,
        height - body_y - margin,
    )
    logs = pygame.Rect(
        margin,
        control.bottom + gap,
        control.width,
        max(120, height - control.bottom - margin - gap),
    )

    draw_status(screen, fonts, data, header)
    draw_stats(screen, fonts, data, stats)
    draw_control(screen, fonts, data, control, controller_name)
    draw_config(screen, fonts, data, config)
    draw_logs(screen, fonts, data, logs)


def choose_joystick(index: int) -> Optional["pygame.joystick.Joystick"]:
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count <= 0:
        return None
    index = max(0, min(index, count - 1))
    joystick = pygame.joystick.Joystick(index)
    joystick.init()
    return joystick


def run_app(args: argparse.Namespace, link: SenderSerial, state) -> None:
    flags = pygame.FULLSCREEN if args.fullscreen else pygame.RESIZABLE
    screen = pygame.display.set_mode((args.width, args.height), flags)
    pygame.display.set_caption("Rotation Sender Deck")
    clock = pygame.time.Clock()
    fonts = Fonts()
    joystick = choose_joystick(args.controller_index)
    deck = DeckInput(
        joystick=joystick,
        lx_axis=args.lx_axis,
        ly_axis=args.ly_axis,
        rt_axis=args.rt_axis,
        deadzone=args.deadzone,
        max_throttle=args.max_throttle,
        invert_y=not args.no_invert_y,
    )
    last_send = 0.0
    send_interval = 1.0 / max(args.send_rate, 1.0)
    controller_name = joystick.get_name() if joystick is not None else ""
    mode = MODE_SETTINGS
    local_throttle_scalar = LOCAL_THROTTLE_SCALAR_MIN
    last_scalar_update = time.monotonic()
    with state.lock:
        state.last_ack = "settings mode"

    while True:
        with state.lock:
            running = state.running
        if not running:
            break

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                with state.lock:
                    state.running = False
            elif event.type == pygame.JOYDEVICEADDED and joystick is None:
                joystick = choose_joystick(args.controller_index)
                deck.joystick = joystick
                controller_name = joystick.get_name() if joystick is not None else ""
            elif event.type == pygame.JOYDEVICEREMOVED:
                joystick = choose_joystick(args.controller_index)
                deck.joystick = joystick
                controller_name = joystick.get_name() if joystick is not None else ""
            elif event.type == pygame.JOYBUTTONDOWN:
                previous_mode = mode
                mode = handle_button_down(event.button, link, state, deck, mode)
                if previous_mode != mode and mode == MODE_SETTINGS:
                    local_throttle_scalar = LOCAL_THROTTLE_SCALAR_MIN
            elif event.type == pygame.JOYBUTTONUP:
                handle_button_up(event.button, deck)
            elif event.type == pygame.JOYHATMOTION and event.value != (0, 0):
                handle_hat(event.value, state, mode)
            elif event.type == pygame.KEYDOWN:
                previous_mode = mode
                mode = handle_key_down(event.key, link, state, deck, mode)
                if previous_mode != mode and mode == MODE_SETTINGS:
                    local_throttle_scalar = LOCAL_THROTTLE_SCALAR_MIN
            elif event.type == pygame.KEYUP:
                handle_key_up(event.key, deck)

        if deck.shutdown_requested(time.monotonic()):
            with state.lock:
                state.running = False
            break

        scalar_now = time.monotonic()
        scalar_dt = max(0.0, scalar_now - last_scalar_update)
        last_scalar_update = scalar_now
        if mode == MODE_CONTROL:
            local_throttle_scalar = update_local_throttle_scalar(
                local_throttle_scalar,
                deck.boost_held(),
                scalar_dt,
            )
        else:
            local_throttle_scalar = LOCAL_THROTTLE_SCALAR_MIN

        control_changed = update_control_from_deck(state, deck, mode, local_throttle_scalar)
        data = snapshot(state)
        data["max_throttle"] = args.max_throttle
        data["mode"] = mode
        data["local_throttle_scalar"] = local_throttle_scalar
        now = time.time()
        if control_changed or (data["send_enabled"] and now - last_send >= send_interval):
            try:
                link.send_control(data["control"])
                last_send = now
            except Exception as exc:
                state.set_error(f"control send failed: {exc}")

        render(screen, fonts, data, controller_name)
        pygame.display.flip()
        clock.tick(FPS)

    try:
        link.send_control(ControlState())
    except Exception:
        pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Steam Deck UI for the ESP32 Sender serial protocol")
    parser.add_argument("-p", "--port", default=None, help="serial port; default uses SENDER_PORT or auto-detects")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument("--send-rate", type=float, default=30.0, help="control packets per second while unpaused")
    parser.add_argument("--fullscreen", action="store_true", help="start fullscreen")
    parser.add_argument("--width", type=int, default=SCREEN_SIZE[0], help="window width")
    parser.add_argument("--height", type=int, default=SCREEN_SIZE[1], help="window height")
    parser.add_argument("--controller-index", type=int, default=0, help="pygame joystick index")
    parser.add_argument("--lx-axis", type=int, default=0, help="left stick X axis index")
    parser.add_argument("--ly-axis", type=int, default=1, help="left stick Y axis index")
    parser.add_argument("--rt-axis", type=int, default=5, help="right trigger axis index")
    parser.add_argument("--deadzone", type=float, default=0.08, help="stick and trigger deadzone")
    parser.add_argument("--max-throttle", type=int, default=1000, help="right trigger full-scale throttle")
    parser.add_argument("--no-invert-y", action="store_true", help="do not invert left-stick Y")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if pygame is None:
        print("pygame is not installed. Install with: python3 -m pip install -r requirements-steam-deck.txt", file=sys.stderr)
        return 1

    port = args.port or find_default_port(args.baud)
    if port is None:
        print("No serial port found. Pass --port /dev/ttyACM0 or /dev/ttyACM1, or set SENDER_PORT.", file=sys.stderr)
        return 2

    pygame.init()
    state = AppState()
    try:
        link = SenderSerial(port, args.baud, state)
    except Exception as exc:
        print(f"Failed to open {port}: {exc}", file=sys.stderr)
        pygame.quit()
        return 1

    try:
        run_app(args, link, state)
    except KeyboardInterrupt:
        pass
    finally:
        with state.lock:
            state.running = False
        try:
            link.send_control(ControlState())
        except Exception:
            pass
        link.close()
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

import sys
import os
import time
import struct
import argparse
import glob
import threading
import fcntl
from collections import deque
from datetime import datetime

import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtWidgets
import serial

# --- Constants & Protocol Definitions ---
START_BYTE = 0xAA

APP_PACKET_TYPE_CONTROL = 0x10
APP_PACKET_TYPE_CONFIG_SET = 0x20
APP_PACKET_TYPE_CONFIG_STATE = 0x21
APP_PACKET_TYPE_STATS = 0x30
APP_PACKET_TYPE_CMD_DUMP = 0x40
APP_PACKET_TYPE_CMD_ACK = 0x41

def calculate_checksum(payload):
    checksum = 0
    for b in payload:
        checksum ^= b
    return checksum

class SerialThread(QtCore.QThread):
    new_stats = QtCore.pyqtSignal(dict)
    new_config = QtCore.pyqtSignal(dict)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.ser = None
        self.log_file = None

        # Thread-safe queue for TX packets
        self.tx_queue = deque()

    def send_packet(self, data):
        self.tx_queue.append(data)

    def parse_hex_string(self, hex_str):
        try:
            return bytes.fromhex(hex_str)
        except ValueError:
            return None

    def run(self):
        log_filename = datetime.now().strftime("meltybrain_log_%Y%m%d_%H%M%S.txt")
        try:
            self.log_file = open(log_filename, 'a')
        except Exception as e:
            print(f"Failed to open log file: {e}")
            return

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.ser.dtr = False
            self.ser.rts = False
            print(f"Opened {self.port} at {self.baud}")
        except Exception as e:
            print(f"Failed to open serial port: {e}")
            self.log_file.close()
            return

        while self.running:
            # 1. Handle Transmit Queue
            while self.tx_queue:
                packet = self.tx_queue.popleft()
                try:
                    self.ser.write(packet)
                    self.log_file.write(f"TX: {packet.hex().upper()}\n")
                    self.log_file.flush()
                except Exception as e:
                    print(f"Serial write error: {e}")

            # 2. Read Receive Buffer
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    if "STATS_DATA:" in line or "CONFIG_DATA:" in line:
                        parts = line.split(":", 1)
                        if len(parts) > 1:
                            raw_val = parts[1].strip()
                            if '\\x1b' in raw_val:
                                raw_val = raw_val.split('\\x1b')[0]
                            hex_str = "".join([c for c in raw_val if c in "0123456789ABCDEFabcdef"])

                            self.log_file.write(f"RX_{parts[0]}: {hex_str.upper()}\n")
                            self.log_file.flush()

                            rx_buf = self.parse_hex_string(hex_str)
                            if rx_buf and len(rx_buf) >= 3:
                                # Validate Checksum (Skip Start Byte 0xAB and Checksum byte)
                                payload = rx_buf[1:-1]
                                calc_sum = calculate_checksum(payload)
                                if calc_sum == rx_buf[-1]:
                                    ptype = payload[0]
                                    if ptype == APP_PACKET_TYPE_STATS and len(payload) == 30:
                                        # Parse stats_packet_t (30 bytes according to struct '<BffibfffI')
                                        unpacked = struct.unpack('<BffibfffI', payload)

                                        stats = {
                                            'type': unpacked[0],
                                            'rssi_mean': unpacked[1],
                                            'rssi_var': unpacked[2],
                                            'pkts_per_sec': unpacked[3],
                                            'last_rssi': unpacked[4],
                                            'rotation_rate': unpacked[5],
                                            'vector_x': unpacked[6],
                                            'vector_y': unpacked[7],
                                            'autocorrelation_time': unpacked[8]
                                        }
                                        self.new_stats.emit(stats)

                                    elif ptype == APP_PACKET_TYPE_CONFIG_STATE and len(payload) == 26:
                                        # Parse app_config_packet_t (26 bytes according to struct '<BBBBBHHffHHfB')
                                        # struct: type(1), dshot_a(1), dshot_b(1), led(1), src(1), step_lag(2), step_win(2),
                                        # thr_mult(4), trans_mult(4), corr_win(2), smooth_win(2), phase_off(4), trans_method(1)
                                        # Actually: B(1) + B(1) + B(1) + B(1) + B(1) + H(2) + H(2) + f(4) + f(4) + H(2) + H(2) + f(4) + B(1) = 26 bytes
                                        unpacked = struct.unpack('<BBBBBHHffHHfB', payload)
                                        config = {
                                            'dshot_pin_a': unpacked[1],
                                            'dshot_pin_b': unpacked[2],
                                            'led_pin': unpacked[3],
                                            'rotation_source': unpacked[4],
                                            'step_lag': unpacked[5],
                                            'step_window': unpacked[6],
                                            'throttle_multiplier': unpacked[7],
                                            'translation_multiplier': unpacked[8],
                                            'correlation_window': unpacked[9],
                                            'smoothing_window': unpacked[10],
                                            'phase_offset': unpacked[11],
                                            'translation_method': unpacked[12]
                                        }
                                        self.new_config.emit(config)
                    else:
                        print(f"DEV: {line}")
                        self.log_file.write(f"DEV: {line}\n")
                        self.log_file.flush()
            except Exception as e:
                print(f"Serial read/parse error: {e}")

            time.sleep(0.01)

        if self.ser:
            self.ser.close()
        if self.log_file:
            self.log_file.close()

    def stop(self):
        self.running = False
        self.wait()

class GamepadThread(QtCore.QThread):
    new_input = QtCore.pyqtSignal(float, float, float) # throttle, vx, vy

    def __init__(self, device_path=""):
        super().__init__()
        self.running = True
        self.device_path = device_path

        # Gamepad state
        self.axis_data = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}

    def find_joystick(self):
        if self.device_path and os.path.exists(self.device_path):
            return self.device_path

        # Fallback to scanning
        js_devices = glob.glob('/dev/input/js*')
        if js_devices:
            return js_devices[0]
        return None

    def run(self):
        dev_path = self.find_joystick()
        if not dev_path:
            print("No joystick found.")
            return

        print(f"Reading gamepad input from {dev_path}")
        try:
            with open(dev_path, 'rb') as f:
                # Make file non-blocking to allow checking self.running gracefully
                fd = f.fileno()
                fl = fcntl.fcntl(fd, fcntl.F_GETFL)
                fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

                while self.running:
                    # Linux joystick event struct: uint32 (time), int16 (value), uint8 (type), uint8 (number)
                    try:
                        ev_buf = f.read(8)
                    except BlockingIOError:
                        time.sleep(0.01)
                        continue

                    if ev_buf and len(ev_buf) == 8:
                        time_val, value, ev_type, number = struct.unpack('<IhBB', ev_buf)

                        # Type 0x02 is JS_EVENT_AXIS
                        if ev_type & 0x02:
                            # Normalize value from [-32767, 32767] to [-1.0, 1.0]
                            norm_val = value / 32767.0
                            self.axis_data[number] = norm_val

                            # Map axes: Left stick (0: X, 1: Y), Right trigger (typically 5, sometimes 2 or 4)
                            # Throttle: Right Trigger (Axis 5) goes from -1.0 (unpressed) to 1.0 (fully pressed)
                            # Let's map it to [0.0, 1.0], scale to 0-1024 or whatever control struct needs.
                            # Standard Linux Xbox mapping: Axis 0 (L-X), Axis 1 (L-Y), Axis 5 (RT)

                            vx = self.axis_data.get(0, 0.0)
                            vy = -self.axis_data.get(1, 0.0) # Invert Y so up is positive

                            # Map trigger [-1, 1] to throttle [0, 1024]
                            rt_val = self.axis_data.get(5, -1.0)
                            throttle_pct = (rt_val + 1.0) / 2.0
                            throttle_raw = throttle_pct * 1024.0 # scale based on typical ESC

                            self.new_input.emit(throttle_raw, vx, vy)
        except Exception as e:
            print(f"Gamepad error: {e}")

    def stop(self):
        self.running = False
        self.wait()

# --- MainWindow ---
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port, baud, joystick_dev):
        super().__init__()
        self.setWindowTitle("Meltybrain Control & Dashboard")
        self.resize(1000, 800)

        # Tab widget
        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)

        # Dashboard Tab
        self.dashboard_tab = QtWidgets.QWidget()
        self.dashboard_layout = QtWidgets.QVBoxLayout(self.dashboard_tab)

        self.graph_widget = pg.GraphicsLayoutWidget()
        self.dashboard_layout.addWidget(self.graph_widget)
        self.p_rpm = self.graph_widget.addPlot(title="Live RPM (15s Window)")
        self.p_rpm.setLabel('left', 'RPM')
        self.p_rpm.setLabel('bottom', 'Time', units='s')
        self.rpm_curve = self.p_rpm.plot(pen='y', name="RPM")

        self.tabs.addTab(self.dashboard_tab, "Dashboard")

        # Settings Tab
        self.settings_tab = QtWidgets.QWidget()
        self.settings_layout = QtWidgets.QFormLayout(self.settings_tab)

        # Settings UI elements
        self.config_inputs = {}

        # Define fields based on struct
        fields = {
            'dshot_pin_a': 'spin', 'dshot_pin_b': 'spin', 'led_pin': 'spin',
            'rotation_source': 'spin', 'step_lag': 'spin', 'step_window': 'spin',
            'throttle_multiplier': 'double', 'translation_multiplier': 'double',
            'correlation_window': 'spin', 'smoothing_window': 'spin',
            'phase_offset': 'double', 'translation_method': 'spin'
        }

        for field, ftype in fields.items():
            if ftype == 'spin':
                inp = QtWidgets.QSpinBox()
                inp.setRange(0, 65535)
            else:
                inp = QtWidgets.QDoubleSpinBox()
                inp.setRange(-1000.0, 1000.0)
                inp.setSingleStep(0.1)

            self.config_inputs[field] = inp
            self.settings_layout.addRow(QtWidgets.QLabel(field), inp)

        self.apply_btn = QtWidgets.QPushButton("Apply Settings")
        self.apply_btn.clicked.connect(self.apply_settings)
        self.settings_layout.addRow(self.apply_btn)

        self.tabs.addTab(self.settings_tab, "Settings")

        # --- State ---
        self.rpm_data = deque(maxlen=1500) # Allow up to 100Hz for 15s
        self.time_data = deque(maxlen=1500)
        self.start_time = time.time()

        # --- Threads ---
        self.serial_thread = SerialThread(port, baud)
        self.serial_thread.new_stats.connect(self.update_stats)
        self.serial_thread.new_config.connect(self.update_config)

        self.gamepad_thread = GamepadThread(joystick_dev)
        self.gamepad_thread.new_input.connect(self.send_control)

        self.serial_thread.start()
        self.gamepad_thread.start()

    def update_stats(self, stats):
        # Rotation rate is in radians/second? Or Hz?
        # Assuming rad/s, RPM = (rate * 60) / (2*PI)
        # Assuming the C struct sends it as something, let's just plot it as raw * scaling

        rpm = stats['rotation_rate'] * 9.549296596425384 # rad/s to RPM

        current_time = time.time() - self.start_time

        self.rpm_data.append(rpm)
        self.time_data.append(current_time)

        # Remove old data outside 15s window
        while self.time_data and current_time - self.time_data[0] > 15.0:
            self.time_data.popleft()
            self.rpm_data.popleft()

        self.rpm_curve.setData(list(self.time_data), list(self.rpm_data))

    def update_config(self, config):
        for k, v in config.items():
            if k in self.config_inputs:
                self.config_inputs[k].setValue(v)

    def send_control(self, throttle, vx, vy):
        # Packing: APP_PACKET_TYPE_CONTROL + throttle(H) + vx(f) + vy(f)
        payload = struct.pack('<BHff', APP_PACKET_TYPE_CONTROL, int(throttle), float(vx), float(vy))
        csum = calculate_checksum(payload)
        packet = bytes([START_BYTE]) + payload + bytes([csum])
        self.serial_thread.send_packet(packet)

    def apply_settings(self):
        # Build app_config_packet_t
        try:
            payload = struct.pack('<BBBBBHHffHHfB',
                APP_PACKET_TYPE_CONFIG_SET,
                int(self.config_inputs['dshot_pin_a'].value()),
                int(self.config_inputs['dshot_pin_b'].value()),
                int(self.config_inputs['led_pin'].value()),
                int(self.config_inputs['rotation_source'].value()),
                int(self.config_inputs['step_lag'].value()),
                int(self.config_inputs['step_window'].value()),
                float(self.config_inputs['throttle_multiplier'].value()),
                float(self.config_inputs['translation_multiplier'].value()),
                int(self.config_inputs['correlation_window'].value()),
                int(self.config_inputs['smoothing_window'].value()),
                float(self.config_inputs['phase_offset'].value()),
                int(self.config_inputs['translation_method'].value())
            )
            csum = calculate_checksum(payload)
            packet = bytes([START_BYTE]) + payload + bytes([csum])
            self.serial_thread.send_packet(packet)
            print("Settings applied and sent.")
        except Exception as e:
            print(f"Error packing settings: {e}")

    def closeEvent(self, event):
        self.serial_thread.stop()
        self.gamepad_thread.stop()
        event.accept()

def main():
    parser = argparse.ArgumentParser(description="Steam Deck Meltybrain UI")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port of Sender ESP")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--joystick", default="", help="Joystick device path (e.g. /dev/input/js0)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOption('background', 'k')
    pg.setConfigOption('foreground', 'w')

    window = MainWindow(args.port, args.baud, args.joystick)
    window.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()

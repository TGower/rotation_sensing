import sys
import serial
import struct
import time
import argparse
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import threading
from collections import deque

# Constants from C code
CSI_LOG_MAGIC = 0xDEADBEEF
PACKET_SIZE = 131 
HEADER_SIZE = 37
CSI_DATA_SIZE = 94 # 47 subcarriers * 2 bytes (I, Q)

# Packet structure: <I (magic) q (local_ts) q (sender_ts) h (x) h (y) h (z) b (rssi) H (csi_len) q (csi_ts)
HEADER_STRUCT = "<IqqhhhbHq"

class CSILoggerThread(QtCore.QThread):
    new_data = QtCore.pyqtSignal(list, list) # deltas, phases_list

    def __init__(self, port, baud, history_len=4000):
        super().__init__()
        self.port = port
        self.baud = baud
        self.history_len = history_len
        self.running = True
        
        self.deltas = deque(maxlen=history_len)
        self.phases = [deque(maxlen=history_len) for _ in range(10)]
        self.last_ts = None
        self.subcarrier_indices = np.linspace(0, 46, 10, dtype=int)

    def run(self):
        print(f"Opening port {self.port} at {self.baud}...")
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            # Reset sequence
            print("Resetting ESP32...")
            ser.dtr = False
            ser.rts = True
            time.sleep(0.1)
            ser.rts = False
            ser.dtr = False
            time.sleep(1.0)
        except Exception as e:
            print(f"Error opening port: {e}")
            return

        print("Listening for data...")
        buffer = bytearray()
        while self.running:
            try:
                data = ser.read(4096)
                if not data:
                    continue
                
                buffer.extend(data)
                
                while len(buffer) >= PACKET_SIZE:
                    magic_idx = buffer.find(struct.pack("<I", CSI_LOG_MAGIC))
                    if magic_idx == -1:
                        if len(buffer) > 4:
                            buffer = buffer[-3:]
                        break
                    
                    if magic_idx > 0:
                        buffer = buffer[magic_idx:]
                    
                    if len(buffer) < PACKET_SIZE:
                        break
                    
                    packet_data = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]
                    
                    self.parse_packet(packet_data)
                    
            except Exception as e:
                print(f"Error reading serial: {e}")
                break
        
        ser.close()

    def parse_packet(self, data):
        header_data = data[:HEADER_SIZE]
        csi_data_raw = data[HEADER_SIZE:HEADER_SIZE+CSI_DATA_SIZE]
        
        try:
            unpacked = struct.unpack(HEADER_STRUCT, header_data)
            local_ts = unpacked[1]
            
            p_vals = []
            for idx in self.subcarrier_indices:
                i_val = int.from_bytes(csi_data_raw[idx*2 : idx*2+1], byteorder='little', signed=True)
                q_val = int.from_bytes(csi_data_raw[idx*2+1 : idx*2+2], byteorder='little', signed=True)
                p_vals.append(np.arctan2(q_val, i_val))
            
            if self.last_ts is not None:
                delta = local_ts - self.last_ts
                if 0 < delta < 1000000:
                    self.deltas.append(delta)
                else:
                    self.deltas.append(0)
            else:
                self.deltas.append(0)
            
            self.last_ts = local_ts
            for i, p in enumerate(p_vals):
                self.phases[i].append(p)
                
            # Emit signal for UI update (batching would be better, but let's try direct first)
            # To avoid overwhelming UI, we could emit at a fixed rate, but PyQt can handle this
            self.new_data.emit(list(self.deltas), [list(p) for p in self.phases])
                    
        except Exception:
            pass

    def stop(self):
        self.running = False
        self.wait()

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port, baud):
        super().__init__()
        self.setWindowTitle("Real-time CSI Monitoring Dashboard (PyQtGraph)")
        self.resize(1000, 800)
        
        # Central widget and layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)
        
        # PyQtGraph widget
        self.win = pg.GraphicsLayoutWidget(show=True)
        layout.addWidget(self.win)
        
        # Delta Plot
        self.p_delta = self.win.addPlot(title="Packet Transmission Stability")
        self.p_delta.setLabel('left', 'Delta', units='us')
        self.p_delta.setYRange(450, 550)
        self.p_delta.addLegend()
        self.curve_delta = self.p_delta.plot(pen='#ff4b4b', name="Time Delta")
        self.p_delta.addLine(y=500, pen=pg.mkPen('w', style=QtCore.Qt.PenStyle.DashLine, width=1))
        
        self.win.nextRow()
        
        # Phase Plot
        self.p_phase = self.win.addPlot(title="Subchannel Phase Diagram")
        self.p_phase.setLabel('left', 'Phase', units='rad')
        self.p_phase.setYRange(-np.pi - 0.2, np.pi + 0.2)
        self.p_phase.addLegend(offset=(10, 10), horSpacing=20, verSpacing=10, labelTextSize='8pt', frame=False)
        
        self.curves_phase = []
        colors = [pg.intColor(i, 10) for i in range(10)]
        self.subcarrier_indices = np.linspace(0, 46, 10, dtype=int)
        for i in range(10):
            c = self.p_phase.plot(pen=colors[i], name=f"SC {self.subcarrier_indices[i]}")
            self.curves_phase.append(c)

        # Start background thread
        self.thread = CSILoggerThread(port, baud)
        self.thread.new_data.connect(self.update_plots)
        self.thread.start()

    def update_plots(self, deltas, phases_list):
        self.curve_delta.setData(deltas)
        for i, p in enumerate(phases_list):
            self.curves_phase[i].setData(p)

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

def main():
    parser = argparse.ArgumentParser(description="Interactive CSI Logger (PyQtGraph)")
    parser.add_argument("--port", default="/dev/ttyACM1", help="Serial port")
    parser.add_argument("--baud", type=int, default=2000000, help="Baud rate")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    # Set dark theme for pyqtgraph
    pg.setConfigOption('background', 'k')
    pg.setConfigOption('foreground', 'w')
    
    window = MainWindow(args.port, args.baud)
    window.show()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

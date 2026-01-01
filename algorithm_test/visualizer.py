import sys
import csv
import argparse
import os
import glob
import numpy as np
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl

NUM_SUBCARRIERS = 47

class RunLoader:
    def __init__(self, run_dir):
        self.run_dir = run_dir
        self.files = sorted(glob.glob(os.path.join(run_dir, "run_*.csv")))
        if not self.files:
            print(f"No run files found in {run_dir}!")
            sys.exit(1)
        print(f"Found {len(self.files)} runs.")

    def load_run(self, index):
        if index < 0 or index >= len(self.files):
            return None
            
        filename = self.files[index]
        ts_deltas = []
        rssi = []
        tmag_points = []
        raw_csi_list = []
        
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                last_ts = None
                
                for row in reader:
                    ts = int(row['local_timestamp'])
                    if last_ts is None:
                        delta = 0
                    else:
                        delta = ts - last_ts
                    
                    ts_deltas.append(delta)
                    rssi.append(int(row['rssi']))
                    last_ts = ts
                    
                    # TMAG Data
                    tx = int(row['tmag_x'])
                    ty = int(row['tmag_y'])
                    tz = int(row['tmag_z'])
                    tmag_points.append([tx, ty, tz])

                    # CSI Data
                    frame_csi = []
                    for sc in range(NUM_SUBCARRIERS):
                        i_val = int(row[f'csi_{sc}_i'])
                        q_val = int(row[f'csi_{sc}_q'])
                        frame_csi.append([i_val, q_val])
                    raw_csi_list.append(frame_csi)
                    
            return {
                'filename': os.path.basename(filename),
                'deltas': np.array(ts_deltas),
                'rssi': np.array(rssi),
                'tmag': np.array(tmag_points),
                'csi': np.array(raw_csi_list, dtype=np.int8)
            }
        except Exception as e:
            print(f"Error loading {filename}: {e}")
            return None

class VisualizerWindow(QtWidgets.QMainWindow):
    def __init__(self, loader):
        super().__init__()
        self.loader = loader
        self.setWindowTitle("CSI Run Viewer - 3D & Color")
        self.resize(1800, 1000)

        # Main Layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)

        # --- 1. Top: Polar Grid ---
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        main_layout.addWidget(scroll, stretch=4)
        
        plot_container = QtWidgets.QWidget()
        grid_layout = QtWidgets.QGridLayout(plot_container)
        # Compact Spacing
        grid_layout.setSpacing(2) 
        grid_layout.setContentsMargins(2, 2, 2, 2)
        scroll.setWidget(plot_container)
        
        self.polar_plots = []
        cols = 12 # Wider grid for compacted plots
        for i in range(NUM_SUBCARRIERS):
            row = i // cols
            col = i % cols
            
            p_widget = pg.PlotWidget(title=None) # Remove titles to save space? Keep checks short.
            p_widget.setTitle(f"{i}", size="8pt")
            p_widget.setAspectLocked(True)
            p_widget.setXRange(-128, 128)
            p_widget.setYRange(-128, 128)
            p_widget.showGrid(x=True, y=True, alpha=0.3)
            p_widget.getPlotItem().hideAxis('bottom') # Hide axes for compactness
            p_widget.getPlotItem().hideAxis('left')
            
            # Very small fixed size
            p_widget.setFixedSize(100, 100) 
            
            scatter = pg.ScatterPlotItem(size=3, pen=None)
            p_widget.addItem(scatter)
            self.polar_plots.append(scatter)
            grid_layout.addWidget(p_widget, row, col)

        # --- 2. Middle: Graphs & 3D Plot ---
        graphs_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(graphs_layout, stretch=2)
        
        # Left: RSSI & Delta
        stats_layout = QtWidgets.QVBoxLayout()
        graphs_layout.addLayout(stats_layout, stretch=1)
        
        self.p_rssi = pg.PlotWidget(title="RSSI")
        self.curve_rssi = self.p_rssi.plot(pen='y')
        stats_layout.addWidget(self.p_rssi)
        
        self.p_delta = pg.PlotWidget(title="Delta (us)")
        self.curve_delta = self.p_delta.plot(pen='r')
        stats_layout.addWidget(self.p_delta)
        
        # Right: 3D TMAG
        self.w_3d = gl.GLViewWidget()
        self.w_3d.opts['distance'] = 2000 # Camera distance
        self.w_3d.setWindowTitle('TMAG 3D')
        
        # Grid for reference
        g = gl.GLGridItem()
        g.scale(100, 100, 1)
        self.w_3d.addItem(g)
        
        self.sp_3d = gl.GLScatterPlotItem(pos=np.array([[0,0,0]]), size=5, color=(1,1,1,1), pxMode=True)
        self.w_3d.addItem(self.sp_3d)
        
        graphs_layout.addWidget(self.w_3d, stretch=2)
        
        # --- 3. Bottom: Controls ---
        control_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(control_layout)
        
        self.lbl_info = QtWidgets.QLabel("Run: 0")
        control_layout.addWidget(self.lbl_info)
        
        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(max(0, len(self.loader.files) - 1))
        self.slider.valueChanged.connect(self.load_run_index)
        control_layout.addWidget(self.slider)

        # Load first run
        self.load_run_index(0)

    def load_run_index(self, idx):
        data = self.loader.load_run(idx)
        if data is None:
            return
            
        count = len(data['deltas'])
        self.lbl_info.setText(f"Run: {idx} | File: {data['filename']} | Packets: {count}")
        
        # Update Timelines
        self.curve_rssi.setData(data['rssi'])
        self.curve_delta.setData(data['deltas'])
        
        # Generate Colors (Blue -> Red)
        # Map 0..count-1 to color using a colormap
        # Using pyqtgraph's colormap features
        if count > 0:
            indices = np.linspace(0, 1, count)
            # Simple gradient map: (0,0,255) to (255,0,0)
            colors = np.zeros((count, 4), dtype=np.float32)
            colors[:, 0] = indices # Red increases
            colors[:, 2] = 1.0 - indices # Blue decreases
            colors[:, 3] = 0.5 # Alpha (0.5)
            
            # For ScatterPlotItem brushes (0-255 ints)
            brushes = []
            for i in range(count):
                b = (int(colors[i,0]*255), int(colors[i,1]*255), int(colors[i,2]*255), 150)
                brushes.append(pg.mkBrush(b))
        else:
            colors = np.zeros((0,4))
            brushes = []

        # Update Polar Plots
        for sc in range(NUM_SUBCARRIERS):
            sc_data = data['csi'][:, sc, :]
            i_vals = sc_data[:, 0]
            q_vals = sc_data[:, 1]
            # Use brushes for color coding
            self.polar_plots[sc].setData(i_vals, q_vals, brush=brushes)

        # Update 3D Plot
        # Scale TMAG slightly? It's usually integers.
        # colors for GLScatterPlotItem are normalized 0-1
        self.sp_3d.setData(pos=data['tmag'], color=colors)

def main():
    parser = argparse.ArgumentParser(description="CSI Run Viewer")
    parser.add_argument("--dir", default="cleaned_runs", help="Directory containing run_*.csv files")
    args = parser.parse_args()
    
    if not os.path.exists(args.dir):
        print(f"Directory {args.dir} not found! Please run cleaner.py first.")
        sys.exit(1)

    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    
    loader = RunLoader(args.dir)
    window = VisualizerWindow(loader)
    window.show()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

import sys
import socket
import pandas as pd
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QFrame, QGridLayout, QPushButton,
                             QFileDialog, QTextEdit, QCheckBox, QSizePolicy,
                             QProgressBar, QDoubleSpinBox, QComboBox, QLineEdit, QSpinBox)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Wedge, Polygon
from matplotlib.lines import Line2D
from collections import deque
import time

# ============================================
#  UDP CONFIGURATION
# ============================================
UDP_LISTEN_PORT = 4210
UDP_LISTEN_HOST = "0.0.0.0"

OBSTACLE_MIN_DISTANCE = 50
OBSTACLE_MAX_DISTANCE = 2000
OBSTACLE_NEAR_THRESHOLD = 500
OBSTACLE_CLOSE_THRESHOLD = 200


# ============================================
#  ODOMETRY CALCULATOR
# ============================================
class OdometryCalculator:
    def __init__(self, wheel_diameter_mm=65.0, ticks_per_rev=360, wheelbase_mm=150.0):
        self.wheel_diameter = wheel_diameter_mm
        self.ticks_per_rev = ticks_per_rev
        self.wheelbase = wheelbase_mm / 1000.0
        self.wheel_circumference = np.pi * (wheel_diameter_mm / 1000.0)
        self.meters_per_tick = self.wheel_circumference / ticks_per_rev
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.first_run = True
        self._prev_time = time.time()
        self.velocity = 0.0  # m/s

    def update(self, left_ticks, right_ticks, yaw_deg=None):
        now = time.time()
        dt = now - self._prev_time
        if dt <= 0:
            dt = 0.04

        if self.first_run:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.first_run = False
            self._prev_time = now
            return self.x, self.y, np.rad2deg(self.theta)

        delta_left  = (left_ticks  - self.prev_left_ticks)  * self.meters_per_tick
        delta_right = (right_ticks - self.prev_right_ticks) * self.meters_per_tick
        delta_distance = (delta_left + delta_right) / 2.0

        # Velocity in m/s from encoder deltas
        self.velocity = delta_distance / dt

        if yaw_deg is not None:
            self.theta = np.deg2rad(yaw_deg)
        else:
            self.theta += (delta_right - delta_left) / self.wheelbase

        self.x += delta_distance * np.cos(self.theta)
        self.y += delta_distance * np.sin(self.theta)

        self.prev_left_ticks  = left_ticks
        self.prev_right_ticks = right_ticks
        self._prev_time = now
        return self.x, self.y, np.rad2deg(self.theta)

    def reset(self):
        self.x = self.y = self.theta = 0.0
        self.velocity = 0.0
        self.prev_left_ticks = self.prev_right_ticks = 0
        self.first_run = True
        self._prev_time = time.time()


# ============================================
#  UNIFIED OBSTACLE MAP
# ============================================
class UnifiedObstacleMap:
    def __init__(self, grid_resolution=0.05):
        self.resolution = grid_resolution
        self.scan_points = {}

    def world_to_grid(self, x, y):
        return (int(round(x / self.resolution)), int(round(y / self.resolution)))

    def is_too_close_to_robot(self, obs_x, obs_y, robot_x, robot_y, threshold=0.3):
        return np.sqrt((obs_x - robot_x)**2 + (obs_y - robot_y)**2) < threshold

    def add_scan_point(self, x, y, distance_mm, robot_x, robot_y):
        if self.is_too_close_to_robot(x, y, robot_x, robot_y):
            return False
        self.scan_points[self.world_to_grid(x, y)] = {
            'time': time.time(), 'distance': distance_mm, 'x': x, 'y': y
        }
        return True

    def get_all_scan_points(self):
        if not self.scan_points:
            return np.array([]), np.array([])
        arr = np.array([(d['x'], d['y']) for d in self.scan_points.values()])
        return arr[:, 0], arr[:, 1]

    def get_obstacle_count(self):
        return len(self.scan_points)

    def get_nearest_obstacle_distance(self, robot_x, robot_y):
        if not self.scan_points:
            return None
        min_dist = min(
            np.sqrt((d['x'] - robot_x)**2 + (d['y'] - robot_y)**2)
            for d in self.scan_points.values()
        )
        return int(min_dist * 1000.0)

    def reset(self):
        self.scan_points = {}


# ============================================
#  SLAM STATE TRACKER
# ============================================
class SLAMTracker:
    def __init__(self):
        self.x = self.y = self.yaw = 0.0
        self.roll = self.pitch = self.velocity = 0.0
        self.vx = self.vy = 0.0
        self.prev_x = self.prev_y = self.prev_yaw = 0.0
        self.prev_time = time.time()
        self.trajectory = deque(maxlen=2000)
        self.predicted_x = self.predicted_y = 0.0
        self.distance_traveled = 0.0
        self.obstacle_map = UnifiedObstacleMap()
        self.current_scan_x = []
        self.current_scan_y = []
        self.current_distance = 0
        self.obstacle_warning = "NONE"

    def update_from_sensors(self, x, y, yaw, distance_mm,
                            roll=0.0, pitch=0.0, velocity=0.0):
        now = time.time()
        dt = now - self.prev_time
        if dt > 0:
            self.vx = (x - self.prev_x) / dt
            self.vy = (y - self.prev_y) / dt

        dx, dy = x - self.prev_x, y - self.prev_y
        self.distance_traveled += np.sqrt(dx*dx + dy*dy)

        self.x, self.y, self.yaw = x, y, yaw
        self.roll     = roll
        self.pitch    = pitch
        self.velocity = velocity

        self.trajectory.append((x, y, yaw, now))
        self.predicted_x = x + self.vx * 0.1
        self.predicted_y = y + self.vy * 0.1
        self.prev_x, self.prev_y, self.prev_yaw = x, y, yaw
        self.prev_time = now

        self.current_scan_x = []
        self.current_scan_y = []
        self.current_distance = distance_mm

        if OBSTACLE_MIN_DISTANCE < distance_mm < OBSTACLE_MAX_DISTANCE:
            dist_m  = distance_mm / 1000.0
            yaw_rad = np.deg2rad(yaw)
            obs_x   = x + dist_m * np.cos(yaw_rad)
            obs_y   = y + dist_m * np.sin(yaw_rad)
            self.current_scan_x = [obs_x]
            self.current_scan_y = [obs_y]
            self.obstacle_map.add_scan_point(obs_x, obs_y, distance_mm, x, y)
            if distance_mm < OBSTACLE_CLOSE_THRESHOLD:
                self.obstacle_warning = "CLOSE"
            elif distance_mm < OBSTACLE_NEAR_THRESHOLD:
                self.obstacle_warning = "NEAR"
            else:
                self.obstacle_warning = "NONE"
        else:
            self.obstacle_warning = "NONE"

    def get_trajectory_points(self):
        if not self.trajectory:
            return np.array([]), np.array([])
        traj = np.array(self.trajectory)
        return traj[:, 0], traj[:, 1]

    def get_tof_cloud(self):
        return self.current_scan_x, self.current_scan_y

    def get_all_obstacles(self):
        return self.obstacle_map.get_all_scan_points()

    def get_obstacle_stats(self):
        return {
            'count':           self.obstacle_map.get_obstacle_count(),
            'warning':         self.obstacle_warning,
            'current_distance':self.current_distance,
            'nearest_distance':self.obstacle_map.get_nearest_obstacle_distance(self.x, self.y)
        }

    def reset(self):
        self.__init__()


# ============================================
#  SLAM CSV PLAYER
# ============================================
class SLAMDataPlayer(QThread):
    data_update     = pyqtSignal(dict)
    progress_update = pyqtSignal(int)
    finished        = pyqtSignal()

    def __init__(self, df, playback_speed=1.0):
        super().__init__()
        self.df            = df
        self.playback_speed = playback_speed
        self.is_playing    = False
        self.current_index = 0
        # Detect extended columns
        self.has_roll     = 'roll_deg'    in df.columns
        self.has_pitch    = 'pitch_deg'   in df.columns
        self.has_velocity = 'velocity_ms' in df.columns

    def set_speed(self, speed):
        self.playback_speed = max(0.1, speed)

    def run(self):
        self.is_playing = True
        self.current_index = 0
        total = len(self.df)
        while self.is_playing and self.current_index < total:
            row = self.df.iloc[self.current_index]
            data = {
                'time':         row['time_s'],
                'yaw':          row['yaw_deg'],
                'left_encoder': int(row['left_encoder_ticks']),
                'right_encoder':int(row['right_encoder_ticks']),
                'tof_mm':       int(row['tof_mm']),
                'roll':     float(row['roll_deg'])    if self.has_roll     else 0.0,
                'pitch':    float(row['pitch_deg'])   if self.has_pitch    else 0.0,
                'velocity': float(row['velocity_ms']) if self.has_velocity else 0.0,
            }
            self.data_update.emit(data)
            self.progress_update.emit(int(self.current_index / total * 100))
            if self.current_index < total - 1:
                dt = self.df.iloc[self.current_index + 1]['time_s'] - row['time_s']
                if dt > 0:
                    time.sleep(dt / self.playback_speed)
            self.current_index += 1
        self.is_playing = False
        self.progress_update.emit(100)
        self.finished.emit()

    def stop(self):
        self.is_playing = False


# ============================================
#  UDP LISTENER
# ============================================
class UDPListener(QThread):
    """
    Listens for UDP packets from ESP32 or localhost test script.

    Basic format  (4 fields):  "yaw,left_ticks,right_ticks,tof_mm"
    Extended format (6 fields): "yaw,left_ticks,right_ticks,tof_mm,roll,pitch"

    Examples:
        "45.3,1200,1205,830"
        "45.3,1200,1205,830,3.5,-1.2"
    """
    data_received     = pyqtSignal(str)
    connection_status = pyqtSignal(bool, str)

    def __init__(self, port=UDP_LISTEN_PORT):
        super().__init__()
        self.port    = port
        self.running = False
        self.sock    = None

    def set_port(self, port):
        self.port = port

    def run(self):
        self.running = True
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((UDP_LISTEN_HOST, self.port))
            self.sock.settimeout(1.0)
            self.connection_status.emit(True, f"Listening on UDP port {self.port}")
        except Exception as e:
            self.connection_status.emit(False, f"Failed to bind: {e}")
            return

        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                self.data_received.emit(data.decode('utf-8').strip())
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.connection_status.emit(False, f"Error: {e}")
                break

        try:
            self.sock.close()
        except Exception:
            pass
        self.connection_status.emit(False, "Stopped")

    def stop(self):
        self.running = False
        self.quit()
        self.wait()


# ============================================
#  ATTITUDE INDICATOR (ARTIFICIAL HORIZON)
# ============================================
class AttitudeIndicator(FigureCanvas):
    """
    Mini artificial horizon.
    Blue = sky, Brown = ground, white line = horizon.
    Roll tilts the horizon; pitch shifts it up/down.
    Yellow crosshair = fixed aircraft/robot reference.
    """
    def __init__(self, parent=None):
        fig = Figure(figsize=(1.8, 1.8), dpi=80)
        fig.patch.set_facecolor('#1a1a1a')
        self.ax = fig.add_axes([0, 0, 1, 1])
        super().__init__(fig)
        self.setParent(parent)
        self.setFixedSize(100, 100)
        self._render(0.0, 0.0)

    def update_attitude(self, roll_deg, pitch_deg):
        self._render(roll_deg, pitch_deg)

    def _render(self, roll_deg, pitch_deg):
        import matplotlib.patches as mpatches
        ax = self.ax
        ax.cla()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_facecolor('#1a1a1a')

        roll_rad    = np.deg2rad(roll_deg)
        pitch_norm  = np.clip(pitch_deg / 30.0, -1, 1)
        horizon_y   = pitch_norm * 0.9

        # Sky
        sky = mpatches.Wedge((0, 0), 0.9, 0, 360, facecolor='#1565C0', zorder=1)
        ax.add_patch(sky)

        # Ground (half-circle rotated by roll, offset by pitch)
        offset_x = -np.sin(roll_rad) * horizon_y
        offset_y =  np.cos(roll_rad) * horizon_y
        gnd_pts_x, gnd_pts_y = [], []
        for ang in np.linspace(roll_rad + np.pi, roll_rad + 2 * np.pi, 60):
            gnd_pts_x.append(np.cos(ang) * 0.9 + offset_x)
            gnd_pts_y.append(np.sin(ang) * 0.9 + offset_y)
        gnd = mpatches.Polygon(list(zip(gnd_pts_x, gnd_pts_y)),
                               facecolor='#5D4037', zorder=2)
        ax.add_patch(gnd)

        # Clip both to circle
        clip = mpatches.Circle((0, 0), 0.9, transform=ax.transData)
        sky.set_clip_path(clip)
        gnd.set_clip_path(clip)

        # Horizon line
        hx = [-np.cos(roll_rad)*0.9 + offset_x,  np.cos(roll_rad)*0.9 + offset_x]
        hy = [-np.sin(roll_rad)*0.9 + offset_y,  np.sin(roll_rad)*0.9 + offset_y]
        ax.plot(hx, hy, color='white', lw=2, zorder=5)

        # Fixed yellow crosshair (robot reference)
        ax.plot([-0.3, -0.1], [0, 0], color='#ffeb3b', lw=3, zorder=10)
        ax.plot([ 0.1,  0.3], [0, 0], color='#ffeb3b', lw=3, zorder=10)
        ax.plot([-0.05, 0.05], [0, 0], color='#ffeb3b', lw=3, zorder=10)
        ax.plot([0, 0], [-0.05, 0.05], color='#ffeb3b', lw=2, zorder=10)

        # Border ring
        ax.add_patch(mpatches.Circle((0, 0), 0.9, fill=False,
                                     edgecolor='#555', lw=2, zorder=15))
        # Labels
        ax.text(0, -1.05, f"R:{roll_deg:+.1f}° P:{pitch_deg:+.1f}°",
                ha='center', va='top', fontsize=6, color='#aaa')
        self.draw()


# ============================================
#  ROBOT MAP VISUALIZATION
# ============================================
class RobotMap(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.fig.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)
        self.fig.patch.set_facecolor('#ffffff')
        self.axes = self.fig.add_subplot(111)
        self.axes.set_facecolor('#f5f5f5')
        super().__init__(self.fig)
        self.setParent(parent)

        self.robot_shape_base = np.array([
            [0.12, 0.00], [-0.09, -0.09], [-0.03, 0.00], [-0.09, 0.09]
        ])
        self.robot_polygon = Polygon(self.robot_shape_base, closed=True,
                                     fc='#333333', ec='#00e676', lw=2, zorder=20)
        self.axes.add_patch(self.robot_polygon)

        self.sensor_beam = Wedge((0, 0), 2.0, 0, 0, fc='#00e676', alpha=0.15, zorder=15)
        self.axes.add_patch(self.sensor_beam)

        self.trajectory_line, = self.axes.plot([], [], color='#00bcd4',
                                               linewidth=2, alpha=0.8, zorder=10)
        self.live_hit, = self.axes.plot([], [], marker='x', color='#ff4081',
                                        markersize=8, markeredgewidth=2,
                                        linestyle='None', zorder=25, alpha=0.9)
        self.obstacle_scatter = self.axes.scatter([], [], c='#2196F3',
                                                  s=4, alpha=0.7, zorder=8)

        legend_elements = [
            Line2D([0],[0], marker='^', color='#ffffff', markerfacecolor='#333333',
                   markeredgecolor='#00e676', markersize=8, label='Robot'),
            Line2D([0],[0], color='#00bcd4', lw=2, label='Path'),
            Line2D([0],[0], marker='x', color='#ffffff', markeredgecolor='#ff4081',
                   linestyle='None', markersize=8, label='Live Scan'),
            Line2D([0],[0], marker='o', color='#ffffff', markerfacecolor='#2196F3',
                   linestyle='None', markersize=6, label='Obstacles/Boundaries')
        ]
        leg = self.axes.legend(handles=legend_elements, loc='upper right',
                               fontsize=8, framealpha=0.9,
                               facecolor='#ffffff', edgecolor='#cccccc')
        for text in leg.get_texts():
            text.set_color("black")
        self.setup_axes()

    def setup_axes(self):
        self.axes.grid(True, linestyle='--', linewidth=0.5, color='#cccccc', alpha=0.7)
        self.axes.tick_params(colors='#333333', labelsize=8)
        for spine in self.axes.spines.values():
            spine.set_edgecolor('#999999')
        self.axes.set_xlim(-3, 3)
        self.axes.set_ylim(-1, 5)
        self.axes.set_aspect('equal', adjustable='box')

    def plot_static_map(self, df):
        if 'object_type' in df.columns and 'x' in df.columns:
            left  = df[df['object_type'] == 'RED_BOUNDARY_LEFT']
            right = df[df['object_type'] == 'RED_BOUNDARY_RIGHT']
            self.axes.scatter(left['x'],  left['y'],  c='#444444', s=10, zorder=1)
            self.axes.scatter(right['x'], right['y'], c='#444444', s=10, zorder=1)
            self.draw()

    def _rotate_and_translate(self, points, x, y, yaw_deg):
        theta = np.deg2rad(yaw_deg)
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta),  np.cos(theta)]])
        return np.dot(points, R.T) + np.array([x, y])

    def update_robot_pos(self, x, y, yaw, pred_x, pred_y,
                         traj_x, traj_y, tof_x, tof_y, obs_x, obs_y,
                         follow_mode=False):
        self.robot_polygon.set_xy(
            self._rotate_and_translate(self.robot_shape_base, x, y, yaw))
        self.sensor_beam.set_center((x, y))
        self.sensor_beam.set_theta1(yaw - 15)
        self.sensor_beam.set_theta2(yaw + 15)
        self.sensor_beam.set_radius(2.0)
        self.trajectory_line.set_data(traj_x, traj_y)
        self.live_hit.set_data(tof_x if tof_x else [], tof_y if tof_y else [])
        if len(obs_x) > 0:
            self.obstacle_scatter.set_offsets(np.column_stack((obs_x, obs_y)))
        else:
            self.obstacle_scatter.set_offsets(np.empty((0, 2)))
        if follow_mode:
            self.axes.set_xlim(x - 2.5, x + 2.5)
            self.axes.set_ylim(y - 2.5, y + 2.5)
        self.draw()


# ============================================
#  MAIN DASHBOARD
# ============================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 SLAM Dashboard — UDP Mode")
        self.resize(1400, 900)
        self.setStyleSheet("background-color: #121212; color: #e0e0e0;")

        self.slam          = SLAMTracker()
        self.odometry      = OdometryCalculator()
        self.slam_player   = None
        self.slam_data_loaded = False
        self.sim_mode      = False
        self.udp_connected = False
        self.packet_count  = 0

        self.sim_timer   = QTimer()
        self.sim_timer.timeout.connect(self.run_simulation)
        self.sim_t       = 0
        self.sim_ticks_l = 0
        self.sim_ticks_r = 0

        # ── FIX 1: render timer — map + attitude draw at 20 FPS only ──────────
        self._needs_render    = False   # flag set when new data arrives
        self._render_timer    = QTimer()
        self._render_timer.timeout.connect(self._render_frame)
        self._render_timer.start(50)    # 20 FPS, runs always

        # ── FIX 4: track previous style states to avoid redundant setStyleSheet
        self._prev_roll_level  = -1
        self._prev_pitch_level = -1
        self._prev_obs_warning = ""
        # ─────────────────────────────────────────────────────────────────────

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # ─────────────────────────────────────────────────
        #  LEFT SIDEBAR
        # ─────────────────────────────────────────────────
        sidebar = QFrame()
        sidebar.setFixedWidth(340)
        sidebar.setStyleSheet(
            "background-color: #1e1e1e; border-right: 1px solid #333; border-radius: 8px;")
        side_layout = QVBoxLayout(sidebar)
        side_layout.setSpacing(3)
        side_layout.setContentsMargins(8, 5, 8, 5)

        # Title
        lbl_title = QLabel("ROBOT TELEMETRY")
        lbl_title.setFont(QFont("Segoe UI", 10, QFont.Bold))
        lbl_title.setStyleSheet("color: #00e676; letter-spacing: 1px; padding:0; margin:0;")
        lbl_title.setAlignment(Qt.AlignCenter)
        lbl_title.setFixedHeight(18)
        side_layout.addWidget(lbl_title)

        # ── UDP CONNECTION PANEL ──────────────────────────
        conn_frame = QFrame()
        conn_frame.setStyleSheet(
            "background-color: #2c2c2c; border-radius: 5px; padding: 2px;")
        conn_layout = QVBoxLayout(conn_frame)
        conn_layout.setSpacing(2)
        conn_layout.setContentsMargins(4, 3, 4, 3)

        lbl_udp = QLabel("📡 UDP CONNECTION")
        lbl_udp.setStyleSheet("color: #2196F3; font-size: 9px; font-weight: bold;")
        lbl_udp.setAlignment(Qt.AlignCenter)
        conn_layout.addWidget(lbl_udp)

        port_row = QHBoxLayout()
        port_row.setSpacing(3)
        lbl_port = QLabel("Port:")
        lbl_port.setStyleSheet("color: #aaa; font-size: 8px;")
        lbl_port.setFixedWidth(28)
        self.port_input = QSpinBox()
        self.port_input.setRange(1024, 65535)
        self.port_input.setValue(UDP_LISTEN_PORT)
        self.port_input.setStyleSheet(
            "background: #1a1a1a; border: 1px solid #444; color: white; "
            "padding: 2px; font-size: 8px;")
        self.port_input.setFixedHeight(20)
        self.btn_connect = QPushButton("▶ Listen")
        self.btn_connect.setStyleSheet(
            "background-color: #2196F3; color: white; padding: 2px; "
            "font-weight: bold; font-size: 8px;")
        self.btn_connect.setFixedHeight(20)
        self.btn_connect.setFixedWidth(65)
        self.btn_connect.clicked.connect(self.toggle_udp_connection)
        port_row.addWidget(lbl_port)
        port_row.addWidget(self.port_input)
        port_row.addWidget(self.btn_connect)
        conn_layout.addLayout(port_row)

        status_row = QHBoxLayout()
        self.lbl_conn_status = QLabel("⚪ Not Listening")
        self.lbl_conn_status.setStyleSheet("color: #999; font-size: 8px;")
        self.lbl_packet_count = QLabel("| Pkts: 0")
        self.lbl_packet_count.setStyleSheet("color: #666; font-size: 8px;")
        status_row.addWidget(self.lbl_conn_status)
        status_row.addStretch()
        status_row.addWidget(self.lbl_packet_count)
        conn_layout.addLayout(status_row)
        side_layout.addWidget(conn_frame)

        # ── FILE BUTTONS ──────────────────────────────────
        btn_layout = QHBoxLayout()
        btn_load = QPushButton("📂 Map")
        btn_load.setStyleSheet(
            "background-color: #2979ff; color: white; padding: 5px; "
            "font-weight: bold; font-size: 10px;")
        btn_load.clicked.connect(self.load_static_map)
        btn_load_slam = QPushButton("📊 Data")
        btn_load_slam.setStyleSheet(
            "background-color: #1976d2; color: white; padding: 5px; "
            "font-weight: bold; font-size: 10px;")
        btn_load_slam.clicked.connect(self.load_slam_data)
        btn_layout.addWidget(btn_load)
        btn_layout.addWidget(btn_load_slam)
        side_layout.addLayout(btn_layout)

        # ── PLAYBACK ──────────────────────────────────────
        pb_frame = QFrame()
        pb_frame.setStyleSheet(
            "background-color: #2c2c2c; border-radius: 5px; padding: 2px;")
        pb_layout = QVBoxLayout(pb_frame)
        pb_layout.setSpacing(2)
        pb_layout.setContentsMargins(4, 3, 4, 3)

        pb_btns = QHBoxLayout()
        self.btn_play = QPushButton("▶")
        self.btn_play.setStyleSheet(
            "background-color: #00c853; padding: 3px; font-size: 10px;")
        self.btn_play.setEnabled(False)
        self.btn_play.clicked.connect(self.play_slam_data)
        self.btn_stop = QPushButton("⏹")
        self.btn_stop.setStyleSheet(
            "background-color: #d50000; padding: 3px; font-size: 10px;")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.stop_slam_data)
        btn_reset = QPushButton("Reset")
        btn_reset.setStyleSheet(
            "background-color: #ff6f00; padding: 3px; font-size: 10px;")
        btn_reset.clicked.connect(self.reset_slam)
        pb_btns.addWidget(self.btn_play)
        pb_btns.addWidget(self.btn_stop)
        pb_btns.addWidget(btn_reset)
        pb_layout.addLayout(pb_btns)

        spd_row = QHBoxLayout()
        spd_row.addWidget(QLabel("Speed:"))
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.1, 20.0)
        self.spin_speed.setSingleStep(0.5)
        self.spin_speed.setValue(1.0)
        self.spin_speed.setSuffix("x")
        self.spin_speed.setStyleSheet(
            "background-color: #444; color: white; border: none; padding: 2px;")
        self.spin_speed.valueChanged.connect(self.update_speed)
        spd_row.addWidget(self.spin_speed)
        pb_layout.addLayout(spd_row)

        self.progress_bar = QProgressBar()
        self.progress_bar.setStyleSheet(
            "QProgressBar{background-color:#222;height:8px;border:1px solid #444}"
            "QProgressBar::chunk{background-color:#00e676}")
        pb_layout.addWidget(self.progress_bar)
        side_layout.addWidget(pb_frame)

        # ── TOGGLES ───────────────────────────────────────
        chk_row = QHBoxLayout()
        self.chk_sim = QCheckBox("Simulate")
        self.chk_sim.setStyleSheet("color: orange; font-size: 10px;")
        self.chk_sim.stateChanged.connect(self.toggle_simulation)
        self.chk_follow = QCheckBox("Follow Robot")
        self.chk_follow.setStyleSheet("color: #00e676; font-size: 10px;")
        self.chk_follow.setChecked(True)
        chk_row.addWidget(self.chk_sim)
        chk_row.addWidget(self.chk_follow)
        side_layout.addLayout(chk_row)

        # ── STAT CARD HELPER ──────────────────────────────
        def create_stat_card(title, value_text, is_green=False):
            card = QFrame()
            card.setStyleSheet("background-color: #1a1a1a; border-radius: 5px;")
            layout = QVBoxLayout(card)
            layout.setContentsMargins(0, 2, 0, 2)
            layout.setSpacing(0)
            lbl_t = QLabel(title)
            lbl_t.setAlignment(Qt.AlignCenter)
            lbl_t.setStyleSheet("color: #666; font-size: 7px; font-weight: bold;")
            lbl_t.setFixedHeight(11)
            lbl_v = QLabel(value_text)
            lbl_v.setAlignment(Qt.AlignCenter)
            color = "#00e676" if is_green else "#ffffff"
            lbl_v.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: bold;")
            lbl_v.setFixedHeight(16)
            layout.addWidget(lbl_t)
            layout.addWidget(lbl_v)
            return card, lbl_v

        # ── OBSTACLE PANEL ────────────────────────────────
        obs_group = QFrame()
        obs_group.setStyleSheet(
            "background-color: #262626; border-radius: 6px; padding: 2px;")
        obs_layout = QVBoxLayout(obs_group)
        obs_layout.setSpacing(2)
        obs_layout.setContentsMargins(4, 2, 4, 2)
        lbl_obs_h = QLabel("⚠ OBSTACLES")
        lbl_obs_h.setAlignment(Qt.AlignCenter)
        lbl_obs_h.setStyleSheet("color: #ff6d00; font-weight: bold; font-size: 8px;")
        lbl_obs_h.setFixedHeight(13)
        obs_layout.addWidget(lbl_obs_h)
        obs_grid = QGridLayout()
        obs_grid.setSpacing(3)
        c_obs_dist, self.lbl_obs_dist = create_stat_card("NEAREST", "-- mm")
        c_obs_warn, self.lbl_obs_warn = create_stat_card("STATUS",  "CLEAR", True)
        obs_grid.addWidget(c_obs_dist, 0, 0)
        obs_grid.addWidget(c_obs_warn, 0, 1)
        obs_layout.addLayout(obs_grid)
        side_layout.addWidget(obs_group)

        # ── SLAM PANEL ────────────────────────────────────
        slam_group = QFrame()
        slam_group.setStyleSheet(
            "background-color: #262626; border-radius: 6px; padding: 2px;")
        slam_layout = QVBoxLayout(slam_group)
        slam_layout.setSpacing(2)
        slam_layout.setContentsMargins(4, 2, 4, 2)
        lbl_slam_h = QLabel("SLAM STATE")
        lbl_slam_h.setAlignment(Qt.AlignCenter)
        lbl_slam_h.setStyleSheet("color: #3d5afe; font-weight: bold; font-size: 8px;")
        lbl_slam_h.setFixedHeight(13)
        slam_layout.addWidget(lbl_slam_h)
        slam_grid = QGridLayout()
        slam_grid.setSpacing(3)
        c_pos_x, self.lbl_pos_x = create_stat_card("POS X", "0.00 m")
        c_pos_y, self.lbl_pos_y = create_stat_card("POS Y", "0.00 m")
        slam_grid.addWidget(c_pos_x, 0, 0)
        slam_grid.addWidget(c_pos_y, 0, 1)
        slam_layout.addLayout(slam_grid)
        side_layout.addWidget(slam_group)

        # ── SENSOR PANEL ─────────────────────────────────
        sensor_frame = QFrame()
        sensor_frame.setStyleSheet(
            "background-color: #262626; border-radius: 6px; padding: 2px;")
        sens_layout = QVBoxLayout(sensor_frame)
        sens_layout.setSpacing(2)
        sens_layout.setContentsMargins(4, 2, 4, 2)
        lbl_sens_h = QLabel("SENSORS")
        lbl_sens_h.setAlignment(Qt.AlignCenter)
        lbl_sens_h.setStyleSheet("color: #00bcd4; font-weight: bold; font-size: 8px;")
        lbl_sens_h.setFixedHeight(13)
        sens_layout.addWidget(lbl_sens_h)
        sens_grid = QGridLayout()
        sens_grid.setSpacing(3)
        c_yaw,      self.lbl_yaw      = create_stat_card("YAW",      "0.0°")
        c_dist,     self.lbl_dist     = create_stat_card("ToF RAW",  "0 mm",     True)
        c_traveled, self.lbl_traveled = create_stat_card("TRAVELED", "0.00 m")
        c_vel,      self.lbl_vel      = create_stat_card("VELOCITY", "0.000 m/s", True)
        sens_grid.addWidget(c_yaw,      0, 0)
        sens_grid.addWidget(c_dist,     0, 1)
        sens_grid.addWidget(c_traveled, 1, 0)
        sens_grid.addWidget(c_vel,      1, 1)
        sens_layout.addLayout(sens_grid)
        side_layout.addWidget(sensor_frame)

        # ── IMU PANEL (Roll, Pitch + Attitude Indicator) ──
        imu_frame = QFrame()
        imu_frame.setStyleSheet(
            "background-color: #262626; border-radius: 6px; padding: 2px;")
        imu_layout = QVBoxLayout(imu_frame)
        imu_layout.setSpacing(2)
        imu_layout.setContentsMargins(4, 2, 4, 2)
        lbl_imu_h = QLabel("IMU — ROLL / PITCH")
        lbl_imu_h.setAlignment(Qt.AlignCenter)
        lbl_imu_h.setStyleSheet("color: #ff9800; font-weight: bold; font-size: 8px;")
        lbl_imu_h.setFixedHeight(13)
        imu_layout.addWidget(lbl_imu_h)

        imu_body = QHBoxLayout()
        imu_body.setSpacing(4)

        # Left: Roll & Pitch numeric cards (stacked)
        imu_nums = QGridLayout()
        imu_nums.setSpacing(3)
        c_roll,  self.lbl_roll  = create_stat_card("ROLL",  "0.00°")
        c_pitch, self.lbl_pitch = create_stat_card("PITCH", "0.00°")
        imu_nums.addWidget(c_roll,  0, 0)
        imu_nums.addWidget(c_pitch, 1, 0)
        imu_body.addLayout(imu_nums)

        # Right: Attitude indicator widget
        self.attitude_indicator = AttitudeIndicator()
        self.attitude_indicator.setFixedSize(100, 100)
        imu_body.addWidget(self.attitude_indicator)

        imu_layout.addLayout(imu_body)
        side_layout.addWidget(imu_frame)

        # ── LOG BOX (always visible, no stretch above it) ─
        self.log_box = QTextEdit()
        self.log_box.setFixedHeight(70)
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet(
            "font-family: Consolas; font-size: 8px; color: #aaa; "
            "background: #121212; border: 1px solid #333;")
        side_layout.addWidget(self.log_box)
        main_layout.addWidget(sidebar)

        # ── MAP VIEW ──────────────────────────────────────
        self.map_view = RobotMap(self)
        self.map_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        main_layout.addWidget(self.map_view)

        # ── UDP LISTENER ──────────────────────────────────
        self.udp_listener = UDPListener(port=UDP_LISTEN_PORT)
        self.udp_listener.data_received.connect(self.handle_udp_packet)
        self.udp_listener.connection_status.connect(self.update_connection_status)

        self.log_box.append("✓ SLAM Dashboard Ready (UDP Mode)")
        self.log_box.append("✓ Roll / Pitch / Velocity support added")
        self.log_box.append("ℹ Basic:    yaw,left,right,tof")
        self.log_box.append("ℹ Extended: yaw,left,right,tof,roll,pitch")

    # ─────────────────────────────────────────────────
    #  FIX 1 & 2: DEDICATED RENDER FRAME (20 FPS)
    #  Called by _render_timer every 50ms.
    #  Map + attitude only draw when new data has arrived (_needs_render flag).
    # ─────────────────────────────────────────────────
    def _render_frame(self):
        if not self._needs_render:
            return                          # no new data — skip entirely
        self._needs_render = False

        traj_x, traj_y = self.slam.get_trajectory_points()
        tof_x,  tof_y  = self.slam.get_tof_cloud()
        obs_x,  obs_y  = self.slam.get_all_obstacles()

        self.map_view.update_robot_pos(
            self.slam.x, self.slam.y, self.slam.yaw,
            self.slam.predicted_x, self.slam.predicted_y,
            traj_x, traj_y, tof_x, tof_y, obs_x, obs_y,
            follow_mode=self.chk_follow.isChecked()
        )
        # Attitude indicator redraws here too — not on every packet
        self.attitude_indicator.update_attitude(self.slam.roll, self.slam.pitch)

    # ─────────────────────────────────────────────────
    #  UDP CONNECTION CONTROL
    # ─────────────────────────────────────────────────
    def toggle_udp_connection(self):
        if not self.udp_connected:
            port = self.port_input.value()
            self.udp_listener.set_port(port)
            self.udp_listener.start()
            self.btn_connect.setText("⏹ Stop")
            self.btn_connect.setStyleSheet(
                "background-color: #d50000; color: white; padding: 2px; "
                "font-weight: bold; font-size: 8px;")
            self.port_input.setEnabled(False)
            self.log_box.append(f"⏳ Starting UDP listener on port {port}...")
        else:
            self.udp_listener.stop()
            self.udp_connected = False
            self.btn_connect.setText("▶ Listen")
            self.btn_connect.setStyleSheet(
                "background-color: #2196F3; color: white; padding: 2px; "
                "font-weight: bold; font-size: 8px;")
            self.port_input.setEnabled(True)
            self.lbl_conn_status.setText("⚪ Not Listening")
            self.lbl_conn_status.setStyleSheet("color: #999; font-size: 8px;")
            self.log_box.append("✓ UDP listener stopped")

    def update_connection_status(self, connected, message):
        self.udp_connected = connected
        if connected:
            self.lbl_conn_status.setText("🟢 Listening")
            self.lbl_conn_status.setStyleSheet(
                "color: #00e676; font-size: 8px; font-weight: bold;")
            self.log_box.append(f"✓ {message}")
        else:
            self.lbl_conn_status.setText(f"🔴 {message}")
            self.lbl_conn_status.setStyleSheet("color: #ff1744; font-size: 8px;")

    # ─────────────────────────────────────────────────
    #  UDP PACKET HANDLER
    # ─────────────────────────────────────────────────
    def handle_udp_packet(self, data_str):
        """
        Basic format  (4 fields): "yaw,left_ticks,right_ticks,tof_mm"
        Extended format(6 fields): "yaw,left_ticks,right_ticks,tof_mm,roll,pitch"
        """
        if not data_str or "ERROR" in data_str:
            return
        try:
            parts = data_str.split(',')
            if len(parts) < 4:
                return
            yaw         = float(parts[0])
            left_ticks  = int(parts[1])
            right_ticks = int(parts[2])
            dist        = int(parts[3])
            roll        = float(parts[4]) if len(parts) > 4 else 0.0
            pitch       = float(parts[5]) if len(parts) > 5 else 0.0

            self.packet_count += 1
            self.lbl_packet_count.setText(f"| Pkts: {self.packet_count}")

            x, y, _ = self.odometry.update(left_ticks, right_ticks, yaw)
            velocity = self.odometry.velocity

            self.lbl_yaw.setText(f"{yaw:.1f}°")
            self.lbl_dist.setText(f"{dist} mm")
            self.process_slam_update(x, y, yaw, dist,
                                     roll=roll, pitch=pitch, velocity=velocity)

        except (ValueError, IndexError):
            self.log_box.append(f"✗ Bad packet: {data_str[:30]}")

    # ─────────────────────────────────────────────────
    #  FILE LOADING
    # ─────────────────────────────────────────────────
    def load_static_map(self):
        fname, _ = QFileDialog.getOpenFileName(self, 'Open Map CSV', '.', "CSV (*.csv)")
        if fname:
            try:
                self.map_view.plot_static_map(pd.read_csv(fname))
                self.log_box.append("✓ Map Loaded")
            except Exception as e:
                self.log_box.append(f"✗ Error: {e}")

    def load_slam_data(self):
        fname, _ = QFileDialog.getOpenFileName(self, 'Open CSV', '.', "CSV (*.csv)")
        if fname:
            try:
                df = pd.read_csv(fname)
                self.slam_df = df
                self.slam_data_loaded = True
                self.btn_play.setEnabled(True)
                extras = [c for c in ['roll_deg','pitch_deg','velocity_ms'] if c in df.columns]
                extra_str = f" (+{','.join(extras)})" if extras else " (basic)"
                self.log_box.append(f"✓ Loaded {len(df)} rows{extra_str}")
            except Exception as e:
                self.log_box.append(f"✗ Error: {e}")

    # ─────────────────────────────────────────────────
    #  CSV PLAYBACK
    # ─────────────────────────────────────────────────
    def play_slam_data(self):
        if not self.slam_data_loaded:
            return
        self.reset_slam()
        self.slam_player = SLAMDataPlayer(self.slam_df, self.spin_speed.value())
        self.slam_player.data_update.connect(self.handle_csv_update)
        self.slam_player.progress_update.connect(self.progress_bar.setValue)
        self.slam_player.finished.connect(self.on_playback_finished)
        self.slam_player.start()
        self.btn_play.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.log_box.append("▶ Playing CSV data")

    def update_speed(self):
        if self.slam_player and self.slam_player.isRunning():
            self.slam_player.set_speed(self.spin_speed.value())

    def stop_slam_data(self):
        if self.slam_player:
            self.slam_player.stop()

    def on_playback_finished(self):
        self.btn_play.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.progress_bar.setValue(0)
        self.log_box.append("✓ Playback finished")

    def handle_csv_update(self, data):
        x, y, _ = self.odometry.update(
            data['left_encoder'], data['right_encoder'], data['yaw'])
        velocity = data.get('velocity', self.odometry.velocity)
        self.lbl_yaw.setText(f"{data['yaw']:.1f}°")
        self.lbl_dist.setText(f"{data.get('tof_mm', 0)} mm")
        self.process_slam_update(
            x, y, data['yaw'], data.get('tof_mm', 0),
            roll=data.get('roll', 0.0),
            pitch=data.get('pitch', 0.0),
            velocity=velocity
        )

    # ─────────────────────────────────────────────────
    #  SIMULATION
    # ─────────────────────────────────────────────────
    def toggle_simulation(self, state):
        self.sim_mode = (state == Qt.Checked)
        if self.sim_mode:
            self.sim_timer.start(50)
            self.log_box.append("▶ Simulation started")
        else:
            self.sim_timer.stop()
            self.log_box.append("⏹ Simulation stopped")

    def run_simulation(self):
        self.sim_t += 0.05
        yaw   = (self.sim_t * 20) % 360
        roll  = 5.0 * np.sin(self.sim_t * 1.5)
        pitch = 3.0 * np.cos(self.sim_t * 1.2)
        self.sim_ticks_l += 5
        self.sim_ticks_r += 5
        dist  = int(1000 + 500 * np.sin(self.sim_t))
        packet = (f"{yaw:.1f},{self.sim_ticks_l},{self.sim_ticks_r},"
                  f"{dist},{roll:.2f},{pitch:.2f}")
        self.handle_udp_packet(packet)

    # ─────────────────────────────────────────────────
    #  CORE SLAM UPDATE
    #  FIX 1: map/attitude drawing removed from here.
    #  Data updates SLAM state + labels instantly.
    #  Rendering happens in _render_frame() at 20 FPS.
    # ─────────────────────────────────────────────────
    def process_slam_update(self, x, y, yaw, dist,
                            roll=0.0, pitch=0.0, velocity=0.0):
        self.slam.update_from_sensors(x, y, yaw, dist,
                                      roll=roll, pitch=pitch, velocity=velocity)
        self.update_displays()      # labels update instantly — no matplotlib here
        self._needs_render = True   # signal render timer to draw on next tick

    # ─────────────────────────────────────────────────
    #  DISPLAY UPDATE
    #  FIX 3: setStyleSheet only called when the level actually changes.
    #  FIX 4: obstacle warning style only updates on change.
    # ─────────────────────────────────────────────────
    def update_displays(self):
        self.lbl_pos_x.setText(f"{self.slam.x:.2f} m")
        self.lbl_pos_y.setText(f"{self.slam.y:.2f} m")
        self.lbl_traveled.setText(f"{self.slam.distance_traveled:.2f} m")
        self.lbl_vel.setText(f"{self.slam.velocity:.3f} m/s")

        # ── Roll: only call setStyleSheet when colour level changes ──────────
        ar = abs(self.slam.roll)
        roll_level = 2 if ar > 10 else 1 if ar > 5 else 0
        if roll_level != self._prev_roll_level:
            roll_colors = ["#00e676", "#ffea00", "#ff1744"]
            self.lbl_roll.setStyleSheet(
                f"color: {roll_colors[roll_level]}; font-size: 11px; font-weight: bold;")
            self._prev_roll_level = roll_level
        self.lbl_roll.setText(f"{self.slam.roll:+.2f}°")

        # ── Pitch: same guard ─────────────────────────────────────────────────
        ap = abs(self.slam.pitch)
        pitch_level = 2 if ap > 10 else 1 if ap > 5 else 0
        if pitch_level != self._prev_pitch_level:
            pitch_colors = ["#ffffff", "#ffea00", "#ff1744"]
            self.lbl_pitch.setStyleSheet(
                f"color: {pitch_colors[pitch_level]}; font-size: 11px; font-weight: bold;")
            self._prev_pitch_level = pitch_level
        self.lbl_pitch.setText(f"{self.slam.pitch:+.2f}°")

        # ── Obstacle status: only update style when warning level changes ─────
        stats   = self.slam.get_obstacle_stats()
        nd      = stats['nearest_distance']
        warning = stats['warning']
        self.lbl_obs_dist.setText(f"{nd} mm" if nd is not None else "-- mm")

        if warning != self._prev_obs_warning:
            if warning == "CLOSE":
                self.lbl_obs_warn.setText("DANGER!")
                self.lbl_obs_warn.setStyleSheet(
                    "color: #ff1744; font-size: 11px; font-weight: bold;")
            elif warning == "NEAR":
                self.lbl_obs_warn.setText("CAUTION")
                self.lbl_obs_warn.setStyleSheet(
                    "color: #ffea00; font-size: 11px; font-weight: bold;")
            else:
                self.lbl_obs_warn.setText("CLEAR")
                self.lbl_obs_warn.setStyleSheet(
                    "color: #00e676; font-size: 11px; font-weight: bold;")
            self._prev_obs_warning = warning

    def reset_slam(self):
        self.slam.reset()
        self.odometry.reset()
        self.packet_count = 0
        self.lbl_packet_count.setText("| Pkts: 0")
        # Reset style tracking so colours repaint correctly after reset
        self._prev_roll_level  = -1
        self._prev_pitch_level = -1
        self._prev_obs_warning = ""
        self.attitude_indicator.update_attitude(0.0, 0.0)
        self.update_displays()
        self.log_box.append("✓ SLAM Reset")

    def closeEvent(self, event):
        self._render_timer.stop()
        if self.udp_listener.isRunning():
            self.udp_listener.stop()
        if self.slam_player and self.slam_player.isRunning():
            self.slam_player.stop()
        event.accept()


# ============================================
#  ENTRY POINT
# ============================================
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
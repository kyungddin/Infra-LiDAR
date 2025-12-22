import argparse
import csv
import json
import math
import os
import signal
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple, List, Deque, Dict

try:
    import numpy as np
except Exception:
    print("[ERR] numpy is required.")
    sys.exit(1)

# CAN 통신
try:
    import can  # pip install python-can
except Exception:
    can = None

# =============================
# Terminal Colors
# =============================
class Colors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

# Optional deps
try:
    import osqp  # type: ignore
    import scipy.sparse as sp  # type: ignore
except Exception:
    osqp = None
    sp = None
try:
    import yaml  # type: ignore
except Exception:
    yaml = None

# =============================
# Optimized inline functions
# =============================
_RAD2DEG = 180.0 / math.pi
_DEG2RAD = math.pi / 180.0

# =============================
# Enums & Dataclasses
# =============================
class State(Enum):
    KEEP_L2 = auto()
    EVADE_L = auto()
    KEEP_L1 = auto()
    RETURN_TO_L2 = auto()
    SAFE_STOP = auto()

class RunMode(Enum):
    TEACHER = auto()
    SHADOW = auto()
    STUDENT = auto()

@dataclass
class VehicleParams:
    L: float = 0.20
    B: float = 0.15
    V_MAX: float = 0.5
    V_MIN: float = 0.0
    DELTA_MAX_DEG: float = 25.0
    DELTA_RATE_MAX_DEGPS: float = 200.0
    A_MAX: float = 2.0
    A_MIN: float = -3.0
    WHEEL_RATE_MAX: float = 3.0
    _delta_max_rad: Optional[float] = None
    
    def __post_init__(self):
        self._delta_max_rad = self.DELTA_MAX_DEG * _DEG2RAD

@dataclass
class RoadParams:
    W: float = 0.30
    MAX_Y_DEVIATION: float = 3.0

@dataclass
class BehaviorParams:
    G_TRIG: float = 0.8
    TTC_TRIG: float = 2.0
    TTC_MIN: float = 1.0
    TTC_EVADE_MIN: float = 1.5
    T_KEEP_L1_MIN: float = 0.5
    G_RETURN_SAFE: float = 1.5
    K_TTC: float = 1.5
    COOLDOWN: float = 0.3
    NEAR_BAND_L1: float = 0.12

@dataclass
class ControllerParams:
    k_ey_keep: float = 1.5
    k_ey_evade: float = 2.0
    # [수정] 복귀용 P 게인 추가
    k_ey_return: float = 0.6
    k_delta_rate: float = 0.05

@dataclass
class AppConfig:
    hz: float = 20.0
    run_mode: RunMode = RunMode.TEACHER
    assume_straight_nominal: int = 1
    comm: str = "dummy"
    hb_period: float = 0.5
    log_dir: str = "./runs"
    export_dataset: int = 0
    dataset_dir: str = "./dataset"
    merge_shards: int = 0
    merged_out: Optional[str] = None
    tau_ms: float = 0.0
    tau_jitter_ms: float = 0.0
    policy_onnx: Optional[str] = None
    policy_ts: Optional[str] = None
    yaml_path: Optional[str] = None
    debug_print: int = 0
    lidar_port: int = 8000
    ir_sensor_spacing: float = 0.04

@dataclass
class InfraROI:
    front_gap: Optional[float] = None
    roi_left_clear: bool = True
    timestamp: float = 0.0

# =============================
# Utils
# =============================
def validate_sensor(value: Optional[float], min_val: float, max_val: float, name: str = "sensor") -> Optional[float]:
    if value is None:
        return None
    if not isinstance(value, (int, float)):
        # print(f"{Colors.YELLOW}[WARN] {name}: Invalid type{Colors.RESET}")
        return None
    if not (min_val <= value <= max_val):
        # print(f"{Colors.YELLOW}[WARN] {name}: Out of range {value:.2f}{Colors.RESET}")
        return None
    return value

def print_state_change(old_state: str, new_state: str):
    color_map = {
        'KEEP_L2': Colors.GREEN,
        'EVADE_L': Colors.YELLOW,
        'KEEP_L1': Colors.CYAN,
        'RETURN_TO_L2': Colors.BLUE,
        'SAFE_STOP': Colors.MAGENTA
    }
    old_color = color_map.get(old_state, '')
    new_color = color_map.get(new_state, '')
    print(f"{Colors.BOLD}[STATE]{Colors.RESET} {old_color}{old_state}{Colors.RESET} -> {new_color}{new_state}{Colors.RESET}")

# =============================
# IR Sensor Processing
# =============================
def ir_to_y_position(ir_left: int, ir_center: int, ir_right: int, sensor_spacing: float = 0.05) -> Optional[float]:
    if ir_center == 1:
        if ir_left == 1 and ir_right == 0:
            return sensor_spacing * 0.5
        elif ir_left == 0 and ir_right == 1:
            return -sensor_spacing * 0.5
        else:
            return 0.0
    elif ir_left == 1 and ir_center == 0 and ir_right == 0:
        return sensor_spacing
    elif ir_right == 1 and ir_center == 0 and ir_left == 0:
        return -sensor_spacing
    else:
        return None

# =============================
# LiDAR WiFi Processing
# =============================
class YDLidarWiFi:
    def __init__(self, port: int = 8000):
        self.port = port
        self.sock = None
        self.points = []
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        
    def start(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('', self.port))
            self.sock.settimeout(0.1)
            self.running = True
            self.thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.thread.start()
            print(f"{Colors.GREEN}[LiDAR] WiFi receiver started on port {self.port}{Colors.RESET}")
        except Exception as e:
            print(f"{Colors.RED}[LiDAR] Failed to start: {e}{Colors.RESET}")
            self.running = False
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.sock:
            self.sock.close()
    
    def _receive_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                points = self._parse_packet(data)
                if points:
                    with self.lock:
                        self.points = points
            except socket.timeout:
                continue
            except Exception as e:
                pass
    
    def _parse_packet(self, data: bytes) -> List[Tuple[float, float, float]]:
        if len(data) < 4: return []
        if data[0] != 0xAA or data[1] != 0x55: return []
        num_points = struct.unpack('<H', data[2:4])[0]
        points = []
        offset = 4
        point_size = 9
        for i in range(num_points):
            if offset + point_size > len(data): break
            x = struct.unpack('<f', data[offset:offset+4])[0]
            y = struct.unpack('<f', data[offset+4:offset+8])[0]
            intensity = data[offset+8]
            points.append((x, y, intensity))
            offset += point_size
        return points
    
    def get_points(self) -> List[Tuple[float, float, float]]:
        with self.lock:
            return self.points.copy()

class InfraLidarProcessor:
    def __init__(self, vehicle_width: float = 0.15, vehicle_length: float = 0.20,
                 lane_width: float = 0.30, lidar_margin: float = 0.05):
        self.vehicle_width = vehicle_width
        self.vehicle_length = vehicle_length
        self.lane_width = lane_width
        self.lidar_margin = lidar_margin
        self.lidar_x_W = 0.0
        self.lidar_y_W = 1.5
        self.lidar_z_W = 1.0
        self.lidar_yaw_W = 0.0
    
    def lidar_to_world(self, points_L: List[Tuple[float, float, float]]) -> np.ndarray:
        if not points_L: return np.array([]).reshape(0, 3)
        pc_L = np.array(points_L)
        yaw = self.lidar_yaw_W
        cos_yaw = math.cos(yaw); sin_yaw = math.sin(yaw)
        R_WL = np.array([[cos_yaw, -sin_yaw, 0], [sin_yaw, cos_yaw, 0], [0, 0, 1]])
        pc_W = (R_WL @ pc_L.T).T
        pc_W[:, 0] += self.lidar_x_W
        pc_W[:, 1] += self.lidar_y_W
        pc_W[:, 2] += self.lidar_z_W
        return pc_W
    
    def world_to_car(self, points_W: np.ndarray, car_x_W: float, car_y_W: float, car_yaw_W: float) -> np.ndarray:
        if points_W.shape[0] == 0: return np.array([]).reshape(0, 2)
        p_shift = points_W[:, :2].copy()
        p_shift[:, 0] -= car_x_W
        p_shift[:, 1] -= car_y_W
        cos_psi = math.cos(car_yaw_W); sin_psi = math.sin(car_yaw_W)
        R_RW = np.array([[cos_psi, sin_psi], [-sin_psi, cos_psi]])
        pc_R = (R_RW @ p_shift.T).T
        return pc_R
    
    def process(self, points_L: List[Tuple[float, float, float]], 
                car_x_W: float, car_y_W: float, car_yaw_W: float) -> Tuple[Optional[float], bool]:
        if not points_L: return None, True
        points_W = self.lidar_to_world(points_L)
        points_R = self.world_to_car(points_W, car_x_W, car_y_W, car_yaw_W)
        if points_R.shape[0] == 0: return None, True
        
        margin = 0.1
        front_mask = (points_R[:, 0] > 0) & (np.abs(points_R[:, 1]) < self.vehicle_width/2 + margin)
        front_gap = float(np.min(np.linalg.norm(points_R[front_mask], axis=1))) if front_mask.any() else None
        
        left_mask = (points_R[:, 0] > 0.3) & (points_R[:, 0] < 1.5) & (points_R[:, 1] > 0.15) & (points_R[:, 1] < 0.6)
        left_clear = not left_mask.any()
        return front_gap, left_clear

class ParamServer:
    def __init__(self, path: Optional[str]):
        self.path = path
        self.mtime = 0.0
        self.data: Dict[str, Dict] = {}
    def load(self) -> bool:
        if self.path and yaml and os.path.isfile(self.path):
            m = os.path.getmtime(self.path)
            if m != self.mtime:
                with open(self.path, 'r', encoding='utf-8') as fp: self.data = yaml.safe_load(fp) or {}
                self.mtime = m
                return True
        return False
    def apply(self, vp, rp, bp, cp) -> bool:
        changed = False
        def upd(obj, name):
            nonlocal changed
            if name in self.data:
                for k, v in self.data[name].items():
                    if hasattr(obj, k): setattr(obj, k, v); changed = True
        upd(vp, 'vehicle'); upd(rp, 'road'); upd(bp, 'behavior'); upd(cp, 'controller')
        return changed

class SignalValidator:
    def __init__(self, timeout: float):
        self.timeout = timeout; self.ts = 0.0; self.v = 0.0
    def is_valid(self, now: float) -> bool: return (now - self.ts) < self.timeout

class RateLimit:
    def __init__(self, max_rate_per_step: float):
        self.max_rate = max_rate_per_step; self.prev = 0.0
    def step(self, desired: float) -> float:
        d = desired - self.prev
        d = max(-self.max_rate, min(self.max_rate, d))
        self.prev += d
        return self.prev

class PoseEstimator:
    def __init__(self, L: float):
        self.L = L; self.x = 0.0; self.y = 0.0; self.psi = 0.0
    def update(self, v: float, delta_rad: float, dt: float):
        self.x += v * math.cos(self.psi) * dt
        self.y += v * math.sin(self.psi) * dt
        self.psi += (v / self.L) * math.tan(delta_rad) * dt

class LineTraceSensor:
    def __init__(self, lane_width: float):
        self.W = lane_width; self.sig_ey = SignalValidator(timeout=0.2)
    def update(self, ey: Optional[float]):
        if ey is not None: self.sig_ey.ts = time.time(); self.sig_ey.v = ey
    def get_ey(self) -> Optional[float]:
        return self.sig_ey.v if self.sig_ey.is_valid(time.time()) else None

class SpeedSensor:
    def __init__(self):
        self.sig = SignalValidator(timeout=0.2); self._v = 1.2
    def update(self, v_meas: float):
        self.sig.ts = time.time(); self.sig.v = v_meas; self._v = v_meas
    def read(self) -> float:
        return self.sig.v if self.sig.is_valid(time.time()) else self._v

# =============================
# Controllers
# =============================
class LateralController:
    def __init__(self, cp: ControllerParams, vp: VehicleParams):
        self.cp = cp; self.vp = vp
    def compute(self, ey: float, mode: str) -> float:
        # [수정] 모드별 게인 선택 (return 추가)
        if mode == 'keep': k = self.cp.k_ey_keep
        elif mode == 'evade': k = self.cp.k_ey_evade
        else: k = self.cp.k_ey_return 
        
        delta_deg = -k * ey * _RAD2DEG
        dmax = self.vp.DELTA_MAX_DEG
        return max(-dmax, min(dmax, delta_deg))

# =============================
# MPC
# =============================
class OSQPMPC:
    def __init__(self, vp: VehicleParams, rp: RoadParams, N: int, dt: float,
                 qy: float, qpsi: float, rdelta: float, rddelta: float):
        if osqp is None or sp is None: raise RuntimeError("osqp/scipy required")
        self.vp = vp; self.rp = rp; self.N = N; self.dt = dt
        self.qy = qy; self.qpsi = qpsi; self.rdelta = rdelta; self.rddelta = rddelta
        self.nx = 2; self.n_ctrl = N - 1; self.n_vars = self.nx * N + self.n_ctrl
        self.y_limit = max(self.rp.MAX_Y_DEVIATION, 10.0); self.psi_limit = math.radians(80.0)
        
        P_diag = []
        for _ in range(N): P_diag.extend([qy, qpsi])
        for i in range(self.n_ctrl): P_diag.append(rdelta if i == 0 else rddelta)
        self.P = sp.diags(P_diag, format='csc')
        
        self.lb = np.full(self.n_vars, -np.inf); self.ub = np.full(self.n_vars, np.inf)
        for i in range(N):
            self.lb[self._idx_y(i)] = -self.y_limit; self.ub[self._idx_y(i)] = self.y_limit
        dmax = vp.DELTA_MAX_DEG * _DEG2RAD
        for k in range(self.n_ctrl):
            idx = self._idx_u(k)
            self.lb[idx] = -dmax; self.ub[idx] = dmax
        self.I_bounds = sp.eye(self.n_vars, format='csc')

    def _idx_y(self, i): return self.nx * i
    def _idx_psi(self, i): return self.nx * i + 1
    def _idx_u(self, i): return self.nx * self.N + i

    def solve(self, y0: float, psi0: float, v0: float, target_y: float) -> Tuple[float, bool]:
        v = max(0.05, min(self.vp.V_MAX, abs(v0)))
        gain_y = v * self.dt
        gain_psi = (v / max(self.vp.L, 1e-3)) * self.dt
        
        rows, cols, data, l_eq, u_eq = [], [], [], [], []
        row_idx = 0
        
        rows.extend([row_idx]); cols.extend([self._idx_y(0)]); data.extend([1.0])
        l_eq.append(y0); u_eq.append(y0); row_idx += 1
        psi_init = max(-self.psi_limit, min(self.psi_limit, psi0))
        rows.extend([row_idx]); cols.extend([self._idx_psi(0)]); data.extend([1.0])
        l_eq.append(psi_init); u_eq.append(psi_init); row_idx += 1
        
        for k in range(self.N - 1):
            rows.extend([row_idx, row_idx, row_idx])
            cols.extend([self._idx_y(k+1), self._idx_y(k), self._idx_psi(k)])
            data.extend([1.0, -1.0, -gain_y])
            l_eq.extend([0.0]); u_eq.extend([0.0]); row_idx += 1
            
            rows.extend([row_idx, row_idx, row_idx])
            cols.extend([self._idx_psi(k+1), self._idx_psi(k), self._idx_u(k)])
            data.extend([1.0, -1.0, -gain_psi])
            l_eq.extend([0.0]); u_eq.extend([0.0]); row_idx += 1
            
        A = sp.vstack([sp.csc_matrix((data, (rows, cols)), shape=(row_idx, self.n_vars)), self.I_bounds], format='csc')
        l = np.concatenate([np.array(l_eq), self.lb])
        u = np.concatenate([np.array(u_eq), self.ub])
        q = np.zeros(self.n_vars)
        for i in range(self.N): q[self._idx_y(i)] = -2.0 * self.qy * target_y
        
        solver = osqp.OSQP()
        solver.setup(P=self.P, q=q, A=A, l=l, u=u, verbose=False, polish=True, eps_abs=1e-5, eps_rel=1e-5, max_iter=8000)
        res = solver.solve()
        
        if res.info.status != 'solved':
            delta_fallback = np.clip(-self.qy * (y0 - target_y) * _RAD2DEG, -self.vp.DELTA_MAX_DEG, self.vp.DELTA_MAX_DEG)
            return delta_fallback, False
        return float(res.x[self._idx_u(0)] * _RAD2DEG), True

# =============================
# Safety & Comm
# =============================
class SafetyGuard:
    def __init__(self, vp, bp, dt):
        self.vp = vp; self.bp = bp; self.dt = dt; self.interventions = 0; self.min_ttc = float('inf')
        self.rate = RateLimit(max_rate_per_step=vp.DELTA_RATE_MAX_DEGPS * dt)
    def check(self, v_cmd, delta_deg, ttc):
        if ttc < self.min_ttc: self.min_ttc = ttc
        v_cmd = max(self.vp.V_MIN, min(self.vp.V_MAX, v_cmd))
        dmax = self.vp.DELTA_MAX_DEG
        delta_cmd = max(-dmax, min(dmax, delta_deg))
        delta_cmd = self.rate.step(delta_cmd)
        if abs(delta_deg - delta_cmd) > 1e-3: self.interventions += 1
        return v_cmd, delta_cmd

class Watchdog:
    def __init__(self, dt):
        self.dt = dt; self.overruns = 0; self.state = 'OK'; self._thr = 0.7 * dt * 1000.0
    def step(self, loop_ms):
        self.overruns = (self.overruns + 1) if loop_ms > self._thr else max(0, self.overruns - 1)
        if self.overruns > 5: self.state = 'DEGRADE'
        if self.overruns > 15: self.state = 'STOP'
        return self.state

class Comm:
    def __init__(self, mode, hb_period, cfg):
        self.mode = mode; self.hb_period = hb_period; self.cfg = cfg; self.ok = True
        self.can_bus = None; self.lidar_wifi = None; self.lidar_proc = None
        self._infra_roi = None; self._infra_lock = threading.Lock()
        self.last_hb = 0.0
        
        if mode == "can" and can:
            try: self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
            except: self.mode = "dummy"
        if cfg.lidar_port > 0:
            self.lidar_wifi = YDLidarWiFi(port=cfg.lidar_port)
            self.lidar_proc = InfraLidarProcessor()
            self.lidar_wifi.start()
            
    def get_line_position(self) -> Optional[float]:
        if self.mode == "can" and self.can_bus:
            msg = self.can_bus.recv(timeout=0.001)
            if msg and msg.arbitration_id == 0x200 and len(msg.data) >= 3:
                return ir_to_y_position(1 if msg.data[0] else 0, 1 if msg.data[1] else 0, 1 if msg.data[2] else 0, self.cfg.ir_sensor_spacing)
        return None

    def get_lidar_data(self, car_x_W, car_y_W, car_yaw_W) -> InfraROI:
        now = time.time()
        if not self.lidar_wifi: return InfraROI(timestamp=now)
        points = self.lidar_wifi.get_points()
        fg, lc = self.lidar_proc.process(points, car_x_W, car_y_W, car_yaw_W)
        roi = InfraROI(front_gap=fg, roi_left_clear=lc, timestamp=now)
        with self._infra_lock: self._infra_roi = roi
        return roi

    def send(self, v_cmd, steer_deg) -> bool:
        if time.time() - self.last_hb >= self.hb_period: self.last_hb = time.time()
        if self.mode == "can" and self.can_bus:
            v_int = max(-32768, min(32767, int(v_cmd * 100)))
            d_int = max(-32768, min(32767, int(steer_deg * 10)))
            v_u = v_int if v_int >= 0 else (1<<16) + v_int
            d_u = d_int if d_int >= 0 else (1<<16) + d_int
            try: self.can_bus.send(can.Message(arbitration_id=0x123, data=[v_u>>8, v_u&0xFF, d_u>>8, d_u&0xFF], is_extended_id=False))
            except: return False
        return True
    
    def close(self):
        if self.can_bus: self.can_bus.shutdown()
        if self.lidar_wifi: self.lidar_wifi.stop()

# =============================
# Main App
# =============================
class App:
    def __init__(self, cfg: AppConfig, vp: VehicleParams, rp: RoadParams, bp: BehaviorParams, cp: ControllerParams):
        self.cfg = cfg; self.vp = vp; self.rp = rp; self.bp = bp; self.cp = cp
        self.dt = 1.0 / cfg.hz
        
        self.pose = PoseEstimator(vp.L)
        self.lines = LineTraceSensor(lane_width=rp.W)
        self.speed = SpeedSensor()
        self.lat = LateralController(cp, vp)
        
        self.mpc = OSQPMPC(vp, rp, N=15, dt=self.dt, qy=100.0, qpsi=25.0, rdelta=0.5, rddelta=2.0)
        
        self.guard = SafetyGuard(vp, bp, dt=self.dt)
        self.watchdog = Watchdog(self.dt)
        self.comm = Comm(cfg.comm, cfg.hb_period, cfg)
        
        self.state = State.KEEP_L2
        self.target_lane = 2
        self.last_state_change = time.time()
        self.delta_filtered = 0.0
        self.v_cmd_prev = 0.0
        self.debug_target_y = 0.0
        self.front_gap = None
        self.infra = InfraROI()
        self.left_clear_override = None
        
        # [수정] 회피 시작 지점 기억용 변수
        self.saved_l2_y = 0.0
        
        self.steer_ratio = 10.0
        self.steer_offset_deg = 0.0
        
        self._stop = False; self._stop_flag = False; self._stop_reason = ""
        self._test_time = None; self._step_count = 0
        self.param_srv = ParamServer(cfg.yaml_path); self.last_yaml_check = 0.0
        
        os.makedirs(cfg.log_dir, exist_ok=True)
        self.fp = open(os.path.join(cfg.log_dir, f"run_{int(time.time())}.csv"), 'w', newline='', encoding='utf-8')
        self.logger = csv.writer(self.fp)
        self.logger.writerow(["t", "state", "x", "y", "v", "front_gap", "ttc", "v_cmd", "delta_deg", "target_y"])
        self.last_log = None
        
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, '_stop', True))

    def _current_time(self) -> float: return self._test_time if self._test_time is not None else time.time()
    
    def set_state(self, new_state: State):
        if new_state != self.state:
            print_state_change(self.state.name, new_state.name)
            self.state = new_state
            self.last_state_change = self._current_time()

    def step_once(self):
        if self._test_time is not None: time.sleep(self.dt)
        t0 = self._current_time()
        
        if self._test_time is None:
            y_line = validate_sensor(self.comm.get_line_position(), -0.6, 0.6)
            if y_line is not None: self.lines.update(y_line)
            lidar_roi = self.comm.get_lidar_data(self.pose.x, self.pose.y, self.pose.psi)
            self.infra = lidar_roi
            self.front_gap = self.infra.front_gap
            left_clear = self.left_clear_override if self.left_clear_override is not None else self.infra.roi_left_clear
        else:
            self.front_gap = self.infra.front_gap
            left_clear = self.left_clear_override if self.left_clear_override is not None else self.infra.roi_left_clear
            
        v_now = self.v_cmd_prev if self.v_cmd_prev > 0.0 else self.vp.V_MAX
        ey_line = self.lines.get_ey()
        ttc = (self.front_gap / max(0.01, v_now)) if self.front_gap else float('inf')
        
        if self.front_gap and ttc < self.bp.TTC_MIN and not left_clear:
            self.set_state(State.SAFE_STOP)
            self._stop_flag = True; self._stop_reason = "TTC_EMERGENCY"
            self.comm.send(0.0, 0.0)
            return

        # =======================
        # State Machine & Control
        # =======================
        v_ref = self.vp.V_MAX
        delta_deg = 0.0
        target_display = 0.0
        
        if self.state == State.KEEP_L2:
            if self.front_gap and self.front_gap < self.bp.G_TRIG and ttc < self.bp.TTC_TRIG and left_clear:
                # [수정] 회피 시작 시점의 Y 좌표 저장
                self.saved_l2_y = self.pose.y
                print(f"{Colors.MAGENTA}[MEMORY] Saved Return Target Y: {self.saved_l2_y:.3f}{Colors.RESET}")
                
                self.set_state(State.EVADE_L)
                self.target_lane = 1
            else:
                if ey_line is not None:
                    if self._test_time is None: self.pose.y = 0.0 + ey_line
                    delta_deg = self.lat.compute(ey_line, 'keep')
                    target_display = 0.0
                else:
                    delta_deg, _ = self.mpc.solve(self.pose.y, self.pose.psi, v_now, 0.0)
                    target_display = 0.0

        elif self.state == State.EVADE_L:
            if ey_line is not None and self.pose.y > 0.5 * self.rp.W:
                if abs(ey_line) < (self.bp.NEAR_BAND_L1 * 0.2): self.set_state(State.KEEP_L1)
            elif (0.002 < (self.pose.y - self.rp.W) < 0.010):
                self.set_state(State.KEEP_L1)
            elif ttc < self.bp.TTC_EVADE_MIN and not left_clear:
                self.set_state(State.KEEP_L1)
            
            target_y = self.rp.W
            delta_mpc, _ = self.mpc.solve(self.pose.y, self.pose.psi, v_now, target_y)
            if ey_line is not None:
                delta_line = self.lat.compute(ey_line, 'evade')
                delta_deg = 0.1 * delta_mpc + 0.9 * delta_line
            else:
                delta_deg = delta_mpc
            target_display = target_y

        elif self.state == State.KEEP_L1:
            if (t0 - self.last_state_change) > self.bp.T_KEEP_L1_MIN:
                if not self.front_gap or self.front_gap > self.bp.G_RETURN_SAFE:
                    self.set_state(State.RETURN_TO_L2)
                    self.target_lane = 2
            
            if ey_line is not None:
                if self._test_time is None: self.pose.y = self.rp.W + ey_line
                delta_deg = self.lat.compute(ey_line, 'keep')
            else:
                delta_deg, _ = self.mpc.solve(self.pose.y, self.pose.psi, v_now, self.rp.W)
            target_display = self.rp.W

        elif self.state == State.RETURN_TO_L2:
            if ey_line is not None:
                if abs(ey_line) < (self.bp.NEAR_BAND_L1 * 0.5) and abs(self.pose.y) < 0.35:
                    self.set_state(State.KEEP_L2)
            elif abs(self.pose.y - self.saved_l2_y) < 0.10: # 저장된 위치 근처 도달 시
                 self.set_state(State.KEEP_L2)

            # [핵심 수정] MPC 미사용, 기억해둔 좌표로 P제어 복귀
            target_y = self.saved_l2_y
            
            # 현재 차의 방향(psi)을 고려해 0.8m 앞의 위치를 예측
            lookahead = 0.8
            y_predict = self.pose.y + lookahead * math.sin(self.pose.psi)
            
            error_y = y_predict - target_y
            
            # P제어 공식: u = -K * error (여기서 error는 예측 위치 오차)
            delta_deg = self.lat.compute(error_y, 'return')
            
            target_display = target_y

        self.delta_filtered = 0.3 * delta_deg + 0.7 * self.delta_filtered
        v_cmd, delta_final = self.guard.check(v_ref, self.delta_filtered, ttc)
        self.pose.update(v_cmd, delta_final * _DEG2RAD, self.dt)
        self.v_cmd_prev = v_cmd
        
        if not self.comm.send(v_cmd, delta_final * self.steer_ratio + self.steer_offset_deg):
            self._stop_flag = True; self._stop_reason = "COMM_FAIL"

        loop_ms = (self._current_time() - t0) * 1000.0
        wd_state = self.watchdog.step(loop_ms)
        if wd_state == 'STOP': self._stop_reason = "WD_BREACH"
        
        self.last_log = {
            "t": t0, "state": self.state.name, "x": self.pose.x, "y": self.pose.y,
            "v": v_now, "front_gap": self.front_gap, "ttc": ttc,
            "v_cmd": v_cmd, "delta_deg": delta_final, "target_y": target_display
        }
        if self._test_time is not None:
            print(f"[STEP] t={self._step_count*self.dt:.2f} {self.state.name:10s} y={self.pose.y:+.3f} tgt={target_display:+.3f} delta={delta_final:+.1f}")
            self._step_count += 1

    def run(self):
        print(f"{Colors.CYAN}[RUN] MPC OFF for Return / Using Saved Y{Colors.RESET}")
        try:
            while not (self._stop or self._stop_flag):
                self.step_once()
                if self.last_log:
                    self.logger.writerow([
                        f"{self.last_log['t']:.3f}", self.last_log['state'],
                        f"{self.last_log['x']:.2f}", f"{self.last_log['y']:.2f}",
                        f"{self.last_log['v']:.2f}", 
                        f"{self.last_log['front_gap']:.2f}" if self.last_log['front_gap'] else "NA",
                        f"{self.last_log['ttc']:.2f}" if self.last_log['ttc']!=float('inf') else "INF",
                        f"{self.last_log['v_cmd']:.2f}", f"{self.last_log['delta_deg']:.1f}",
                        f"{self.last_log['target_y']:.3f}"
                    ])
                    self.fp.flush()
        finally:
            self.fp.close(); self.comm.close()
        print(f"{Colors.GREEN}[RUN] Stopped. Reason: {self._stop_reason}{Colors.RESET}")

# =============================
# CLI
# =============================
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--hz", type=float, default=20.0)
    p.add_argument("--comm", type=str, default="dummy", choices=["dummy", "can"])
    p.add_argument("--test", action="store_true")
    p.add_argument("--lidar_port", type=int, default=8000)
    args = p.parse_args()
    
    cfg = AppConfig(hz=args.hz, comm=args.comm, lidar_port=args.lidar_port)
    vp = VehicleParams()
    rp = RoadParams()
    bp = BehaviorParams()
    cp = ControllerParams()
    
    app = App(cfg, vp, rp, bp, cp)
    if args.test:
        print("[TEST] Running basic connection test...")
        app.step_once()
    else:
        app.run()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rospy
import math
import tf
import numpy as np
import time
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# MPC 모듈 (같은 폴더에 lane_change_mpc_final_8.py 필요)
import lane_change_mpc_final_8 as mpc_core

class HILSimulatorMPC:
    def __init__(self):
        rospy.init_node('hil_mpc_sim')
        
        self.dt = 0.05       
        self.speed = 0.6
        
        # [좌표 정의]
        self.l2_center = 0.175
        self.l1_center = self.l2_center + 0.30
        
        self.start_y = self.l2_center
        self.mpc_target_y = self.l2_center 
        self.evade_target_y = self.l1_center
        
        self.vp = mpc_core.VehicleParams() 
        self.obs_len_x = 0.3 
        self.obs_width_y = 0.2 
        
        self.car_x = -2.5
        self.car_y = self.start_y
        self.car_yaw = 0.0
        self.current_steer = 0.0
        
        # [수정] 복귀 위치 저장용 변수
        self.saved_return_y = self.l2_center
        self.is_evading = False
        self.return_gain_k = 0.1  # 복귀 시 P제어 게인 (사용 안함, 아래 하드코딩된 0.6 사용)
        
        # MPC 초기화
        self.rp = mpc_core.RoadParams(W=0.30)
        self.mpc = mpc_core.OSQPMPC(
            self.vp, self.rp, N=15, dt=self.dt, 
            qy=70.0,
            qpsi=50.0,
            rdelta=1.5,
            rddelta=2.0
        )
        print(f"[MPC] Ready. Start/Return=L2({self.l2_center:.3f}), Trace=L1({self.l1_center:.3f})")
        
        self.control_state = "KEEP_L2"
        self.evade_trigger_dist = 0.4 
        
        self.scan_points_global = []
        self.fixed_obstacle_pos = None 
        self.left_roi_blocked = False
        self.left_roi_count = 0
        self.roi_block_threshold = 20
        self.is_running = False
        self.step_cnt = 0
        
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.cb_scan)
        self.pub_markers = rospy.Publisher('/vis/all', MarkerArray, queue_size=1)
        self.br = tf.TransformBroadcaster()
        
        rospy.sleep(0.5)
        self.publish_static_visuals("READY (Press Enter)")

    def start(self):
        self.is_running = True
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.update_loop)
        print(f"[HIL Sim] Simulation Started! Logs will scroll below:\n")
        # [수정 1] 헤더에 ROI Pts 추가
        print(f"{'STEP':^6} | {'GAP (cm)':^10} | {'ROI Pts':^7} | {'STEER (deg)':^12} | {'STATE':^15} | {'Target Y':^8}")
        print("-" * 75)

    def cb_scan(self, msg):
        points = []
        angle = msg.angle_min
        lidar_yaw_offset = 0.0 
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                gx = r * math.cos(angle + lidar_yaw_offset)
                gy = r * math.sin(angle + lidar_yaw_offset)
                points.append((gx, gy))
            angle += msg.angle_increment
        self.scan_points_global = points
        if not self.is_running: self.publish_tf()

    def process_sensors(self):
        if not self.scan_points_global: return None
        
        if self.fixed_obstacle_pos is None:
            cos_psi = math.cos(self.car_yaw)
            sin_psi = math.sin(self.car_yaw)
            min_dist = 999.0
            nearest = None
            search_w = self.vp.B + 0.1 
            for (gx, gy) in self.scan_points_global:
                dx = gx - self.car_x
                dy = gy - self.car_y
                lx = dx*cos_psi + dy*sin_psi
                ly = -dx*sin_psi + dy*cos_psi
                if 0.1 < lx < 5.0 and abs(ly) < search_w:
                    if lx < min_dist:
                        min_dist = lx
                        nearest = (gx, gy)
            if nearest:
                self.fixed_obstacle_pos = nearest

        front_gap = None
        self.left_roi_blocked = False
        # [수정 2] 매 스텝 초기화 (장애물 없으면 0)
        self.left_roi_count = 0 
        
        if self.fixed_obstacle_pos:
            obs_surface_x = self.fixed_obstacle_pos[0]
            car_front_x = self.car_x + (self.vp.L / 2.0)
            obs_end_x = obs_surface_x + self.obs_len_x
            car_rear_x = self.car_x - (self.vp.L / 2.0)
            
            safe_return_dist = 0.6 
            
            if car_rear_x > obs_end_x + safe_return_dist:
                front_gap = None # 안전거리 확보됨 -> 복귀 신호
            else:
                front_gap = max(0.0, obs_surface_x - car_front_x)
            
            # Left ROI Check
            obs_center_x = self.fixed_obstacle_pos[0] + (self.obs_len_x / 2.0)
            obs_center_y = self.fixed_obstacle_pos[1]
            roi_cx = obs_center_x
            roi_cy = obs_center_y + 0.35 
            
            cnt = 0
            for (gx, gy) in self.scan_points_global:
                if (roi_cx-0.4 < gx < roi_cx+0.4) and (roi_cy-0.25 < gy < roi_cy+0.25):
                    cnt += 1
            self.left_roi_count = cnt
            if cnt >= self.roi_block_threshold:
                self.left_roi_blocked = True
                
        return front_gap

    def calculate_mpc_control(self, front_gap):
        delta_deg = 0.0
        target_y = 0.0
        
        # [로직 변경] 상태 판단 및 위치 저장
        if front_gap is not None and front_gap < self.evade_trigger_dist:
            # === 회피 모드 (EVADE) ===
            
            # [수정] 회피를 '시작'하는 순간의 Y좌표 저장
            if not self.is_evading:
                self.saved_return_y = self.car_y
                self.is_evading = True
                print(f"   [MEMORY] Saved Return Target Y: {self.saved_return_y:.3f}")

            if self.left_roi_blocked:
                self.control_state = "BLOCKED"
                target_y = self.car_y
            else:
                self.control_state = "TRACE_L1"
                target_y = self.evade_target_y
            
            # 회피 중에는 MPC 사용
            rad, success = self.mpc.solve(
                y0 = self.car_y,
                psi0 = self.car_yaw,
                v0 = self.speed, 
                target_y = target_y
            )
            delta_deg = math.degrees(rad)
            delta_deg = rad # mpc.solve returns degree
            
            if not success: self.control_state += "(FAIL)"

        else:
            # === 복귀 모드 (RETURN) ===
            if front_gap is None:
                self.control_state = "RETURNING(P)"
            else:
                self.control_state = "APPROACH"
            
            target_y = self.saved_return_y
            
            # [핵심 수정] 지그재그 방지를 위한 Lookahead 적용
            lookahead_dist = 0.8  
            y_predict = self.car_y + lookahead_dist * math.sin(self.car_yaw)
            
            error_y = y_predict - target_y
            
            k_gain = 0.6
            
            delta_deg = -k_gain * error_y * (180.0 / math.pi)
            
            # 조향각 제한
            max_deg = self.vp.DELTA_MAX_DEG
            delta_deg = max(-max_deg, min(max_deg, delta_deg))
            
            # [상태 전환] 실제 위치 기준으로 판단
            real_dist = self.car_y - target_y
            if abs(real_dist) < 0.10:
                 self.is_evading = False
                 self.control_state = "KEEP_L2"

        return math.radians(delta_deg), target_y

    def update_loop(self, event):
        self.step_cnt += 1
        front_gap = self.process_sensors()
        steer_cmd, tgt_y = self.calculate_mpc_control(front_gap)
        self.current_steer = steer_cmd 
        
        v = self.speed
        self.car_x += v * math.cos(self.car_yaw) * self.dt
        self.car_y += v * math.sin(self.car_yaw) * self.dt
        self.car_yaw += (v / self.vp.L) * math.tan(self.current_steer) * self.dt
        
        self.publish_tf()
        self.publish_visuals(front_gap)
        
        gap_val = f"{front_gap*100:5.1f}" if front_gap is not None else "   -  "
        deg_val = math.degrees(self.current_steer)
        
        if self.step_cnt % 5 == 0:
            # [수정 3] 로그 출력에 ROI Pts 추가
            print(f"{self.step_cnt:6d} | {gap_val}      | {self.left_roi_count:7d} | {deg_val:10.2f}   | {self.control_state:15s} | {tgt_y:.3f}")

    def publish_tf(self):
        now = rospy.Time.now()
        self.br.sendTransform((self.car_x, self.car_y, 0),
                              tf.transformations.quaternion_from_euler(0, 0, self.car_yaw),
                              now, "base_link", "map")
        self.br.sendTransform((0, 0, 0), (0, 0, 0, 1), now, "laser_frame", "map")

    def publish_static_visuals(self, text):
        self.publish_tf()
        self.publish_visuals(None)

    def publish_visuals(self, gap):
        marker_array = MarkerArray()

        # 1. Car Marker
        car = Marker(type=Marker.CUBE, action=Marker.ADD)
        car.header.frame_id = "base_link"
        car.ns = "car"; car.id = 0
        car.scale.x=self.vp.L; car.scale.y=self.vp.B; car.scale.z=0.1
        car.color.a=0.8; car.color.b=1.0; car.pose.orientation.w=1.0
        marker_array.markers.append(car)
        
        # 2. Yellow Trace Line (1차선 중심)
        trace_line = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
        trace_line.header.frame_id = "map"
        trace_line.ns = "trace_line"; trace_line.id = 1
        trace_line.scale.x=0.02
        trace_line.color.a=1.0; trace_line.color.r=1.0; trace_line.color.g=1.0 
        for i in range(-5, 50): 
            trace_line.points.append(Point(i*0.5, self.l1_center, 0))
        trace_line.pose.orientation.w=1.0
        marker_array.markers.append(trace_line)

        # 3. Black Boundary Lines
        boundaries = Marker(type=Marker.LINE_LIST, action=Marker.ADD)
        boundaries.header.frame_id = "map"
        boundaries.ns = "boundaries"; boundaries.id = 4
        boundaries.scale.x = 0.01 
        boundaries.color.a = 1.0; boundaries.color.r = 0.0; boundaries.color.g = 0.0; boundaries.color.b = 0.0
        
        y_l2_right = self.l2_center - 0.15
        y_l2_left  = self.l2_center + 0.15
        y_l1_left  = self.l2_center + 0.45
        
        for i in range(-5, 50):
            x1 = i * 0.5
            x2 = (i + 1) * 0.5
            boundaries.points.append(Point(x1, y_l2_right, 0))
            boundaries.points.append(Point(x2, y_l2_right, 0))
            boundaries.points.append(Point(x1, y_l2_left, 0))
            boundaries.points.append(Point(x2, y_l2_left, 0))
            boundaries.points.append(Point(x1, y_l1_left, 0))
            boundaries.points.append(Point(x2, y_l1_left, 0))
            
        boundaries.pose.orientation.w = 1.0
        marker_array.markers.append(boundaries)

        # 4. Obstacle & ROI
        if self.fixed_obstacle_pos:
            bbox = Marker(type=Marker.CUBE, action=Marker.ADD)
            bbox.header.frame_id = "map"
            bbox.ns = "bbox"; bbox.id = 2
            bbox.pose.position.x = self.fixed_obstacle_pos[0] + (self.obs_len_x / 2.0)
            bbox.pose.position.y = self.fixed_obstacle_pos[1]
            bbox.pose.position.z = 0.15
            bbox.scale.x=self.obs_len_x; bbox.scale.y=self.obs_width_y; bbox.scale.z=0.3
            bbox.color.a=0.6; bbox.color.r=1.0
            bbox.pose.orientation.w=1.0
            marker_array.markers.append(bbox)
            
            roi = Marker(type=Marker.CUBE, action=Marker.ADD)
            roi.header.frame_id = "map"
            roi.ns = "roi"; roi.id = 3
            roi.pose.position.x = bbox.pose.position.x 
            roi.pose.position.y = bbox.pose.position.y + 0.30 
            roi.scale.x=0.8; roi.scale.y=0.5; roi.scale.z=0.1; roi.color.a=0.3
            if self.left_roi_blocked: roi.color.r=1.0
            else: roi.color.g=1.0
            roi.pose.orientation.w=1.0
            marker_array.markers.append(roi)
        
        self.pub_markers.publish(marker_array)
        
if __name__ == '__main__':
    try:
        sim = HILSimulatorMPC()
        print("\n" + "="*60)
        print("  HIL Sim: 1st Lane Trace (+30cm) & Return")
        print("  Control: Trace(MPC) -> Return(P-Control to Saved Y)")
        print(f"  Start: L2 Center (0.175)")
        print(f"  Trace: L1 Center (0.475)")
        print("  Press [Enter] to Start")
        print("="*60)
        input()
        print("\n[Count] Starting in 5 seconds...")
        for i in range(5, 0, -1):
            print(f"  {i}...", end="\r")
            sim.publish_static_visuals(f"Start in {i}...")
            rospy.sleep(1.0)
        sim.start()
        rospy.spin()
    except rospy.ROSInterruptException: pass
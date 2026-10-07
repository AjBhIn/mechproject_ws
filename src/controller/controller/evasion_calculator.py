#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from typing import List, Tuple
import numpy as np

class EvasionCalculator(Node):
    def __init__(self):
        super().__init__('evasion_calculator')

        self.map_frame = 'map'
        self.robot_frame = 'our_bot/base_link'
        self.enemy_frame = 'enemy_bot/base_link'
        self.target_frame = 'dynamic_nav_target'
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.current_state = 'CHASING'
        self.state_sub = self.create_subscription(String, 'our_bot/robot_state', self.state_cb, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/our_bot/global_costmap/costmap_raw', self.costmap_callback, 10)

        self.sample_radii = [0.8, 1.2, 1.8, 2.0]
        self.angle_steps = [(math.cos(math.radians(deg)), math.sin(math.radians(deg))) for deg in range(0, 360, 8)]

        self.map_data, self.map_res = None, 0.05
        self.map_width, self.map_height = 0, 0
        self.map_origin_x, self.map_origin_y = 0.0, 0.0
        self.MAX_SAFE_COST = 100

        self.last_target_x, self.last_target_y = None, None
        self.MIN_UPDATE_DIST = 0.10
        self.memory_bank: List[Tuple[float, float, float]] = []

        self.timer = self.create_timer(0.1, self.control_loop)

    def state_cb(self, msg):
        self.current_state = msg.data

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def costmap_callback(self, msg: OccupancyGrid):
        self.map_data = np.array(msg.data, dtype=np.int16)
        self.map_res = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def get_cell_cost(self, x: float, y: float) -> int:
        if self.map_data is None: return 255
        raw_col = int((x - self.map_origin_x) / self.map_res)
        raw_row = int((y - self.map_origin_y) / self.map_res)
        col = max(0, min(raw_col, self.map_width - 1))
        row = max(0, min(raw_row, self.map_height - 1))
        cost = self.map_data[row * self.map_width + col]
        return 255 if cost == -1 else cost

    def is_path_clear(self, start_x: float, start_y: float, end_x: float, end_y: float) -> bool:
        if self.map_data is None: return False
        dist = math.hypot(end_x - start_x, end_y - start_y)
        step_size = 0.10 if dist <= 0.8 else 0.05
        steps = max(2, int(dist / step_size))
        t = np.linspace(0.0, 1.0, steps)
        px = start_x + t * (end_x - start_x)
        py = start_y + t * (end_y - start_y)
        cols = np.clip(((px - self.map_origin_x) / self.map_res).astype(np.int32), 0, self.map_width - 1)
        rows = np.clip(((py - self.map_origin_y) / self.map_res).astype(np.int32), 0, self.map_height - 1)
        indices = rows * self.map_width + cols
        costs = self.map_data[indices]
        if np.any(costs > self.MAX_SAFE_COST) or np.any(costs == -1): return False
        return True

    def check_memory_bank(self, our_x, our_y, enemy_x, enemy_y):
        if not self.memory_bank: return []
        still_valid_points = []
        current_dist_to_enemy = math.hypot(our_x - enemy_x, our_y - enemy_y)
        for mx, my, myaw in self.memory_bank:
            mem_dist_to_enemy = math.hypot(mx - enemy_x, my - enemy_y)
            if mem_dist_to_enemy <= (current_dist_to_enemy + 0.3): continue
            if not self.is_path_clear(our_x, our_y, mx, my): continue
            if self.get_cell_cost(mx, my) > self.MAX_SAFE_COST: continue
            still_valid_points.append((mx, my, myaw))
        return still_valid_points

    def get_adaptive_weights(self, current_cost, dist_to_enemy):
        w_dist, w_speed, w_turn, w_cost = 10.0, 2.0, 4.0, 0.3
        if current_cost > 30: w_cost, w_turn = 1.0, 0.5
        if dist_to_enemy < 1.2: w_dist, w_speed = 20.0, 4.0
        return {'w_dist': w_dist, 'w_speed': w_speed, 'w_turn': w_turn, 'w_cost': w_cost}

    def evaluate_future_options(self, cand_x, cand_y):
        future_clear_branches, blocked_branches = 0, 0
        MAX_ALLOWED_BLOCKED, probe_radius = 2, 0.5
        for cos_val, sin_val in self.angle_steps[::4]:
            next_x = cand_x + (probe_radius * cos_val)
            next_y = cand_y + (probe_radius * sin_val)
            if self.is_path_clear(cand_x, cand_y, next_x, next_y) and self.get_cell_cost(next_x, next_y) <= self.MAX_SAFE_COST:
                future_clear_branches += 1
            else:
                blocked_branches += 1
                if blocked_branches >= MAX_ALLOWED_BLOCKED: return 0.0
        return future_clear_branches * 1.5

    def get_safe_escape_points(self, robot_x, robot_y):
        safe_points = []
        for r in self.sample_radii:
            for cos_val, sin_val in self.angle_steps:
                cand_x = robot_x + (r * cos_val)
                cand_y = robot_y + (r * sin_val)
                if self.is_path_clear(robot_x, robot_y, cand_x, cand_y):
                    safe_points.append((cand_x, cand_y, r, self.get_cell_cost(cand_x, cand_y)))
        return safe_points

    def get_best_escape_target(self, safe_points, our_x, our_y, our_yaw, enemy_x, enemy_y):
        best_point, highest_score, all_scored_points = None, -float('inf'), []
        weights = self.get_adaptive_weights(self.get_cell_cost(our_x, our_y), math.hypot(our_x - enemy_x, our_y - enemy_y))
        for cand_x, cand_y, r, cost in safe_points:
            score = (math.hypot(cand_x - enemy_x, cand_y - enemy_y) * weights['w_dist']) + (r * weights['w_speed'])
            angle_to_cand = math.atan2(cand_y - our_y, cand_x - our_x)
            score -= abs(math.atan2(math.sin(angle_to_cand - our_yaw), math.cos(angle_to_cand - our_yaw))) * weights['w_turn']
            score -= cost * weights['w_cost']
            if 0 <= cost < 15: score += 12.0
            score += self.evaluate_future_options(cand_x, cand_y)
            all_scored_points.append((score, cand_x, cand_y, angle_to_cand))
            if score > highest_score: highest_score, best_point = score, (cand_x, cand_y, angle_to_cand)
        all_scored_points.sort(key=lambda item: item[0], reverse=True)
        self.memory_bank = [(item[1], item[2], item[3]) for item in all_scored_points[:3]]
        return best_point

    def control_loop(self):
        if self.current_state != 'EVADING':
            self.last_target_x = None
            self.memory_bank.clear()
            return

        try:
            tf_global = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            tf_enemy_global = self.tf_buffer.lookup_transform(self.map_frame, self.enemy_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return

        global_x, global_y = tf_global.transform.translation.x, tf_global.transform.translation.y
        our_yaw = self.get_yaw_from_quaternion(tf_global.transform.rotation)
        enemy_global_x, enemy_global_y = tf_enemy_global.transform.translation.x, tf_enemy_global.transform.translation.y

        valid_cached_points = self.check_memory_bank(global_x, global_y, enemy_global_x, enemy_global_y)
        if valid_cached_points:
            target_x, target_y, target_yaw = valid_cached_points[0]
        else:
            safe_points = self.get_safe_escape_points(global_x, global_y)
            if not safe_points: return
            best_target = self.get_best_escape_target(safe_points, global_x, global_y, our_yaw, enemy_global_x, enemy_global_y)
            if best_target is None: return
            target_x, target_y, target_yaw = best_target
        
        if self.last_target_x is not None:
            if math.hypot(target_x - self.last_target_x, target_y - self.last_target_y) < self.MIN_UPDATE_DIST: return

        self.last_target_x, self.last_target_y = target_x, target_y

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.target_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = target_x, target_y, 0.0
        t.transform.rotation.z, t.transform.rotation.w = math.sin(target_yaw / 2.0), math.cos(target_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EvasionCalculator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
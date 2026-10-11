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
        self.via_frame = 'dynamic_nav_target_via'
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.current_state = 'CHASING'
        self.state_sub = self.create_subscription(String, 'our_bot/robot_state', self.state_cb, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/our_bot/global_costmap/costmap', self.costmap_callback, 10)

        # Immediate reach radii (0.7m to 1.5m) to ensure fast, direct escape vectors
        self.sample_radii = [0.7, 1.1, 1.5]
        self.angle_steps = [(math.cos(math.radians(deg)), math.sin(math.radians(deg))) for deg in range(0, 360, 10)]

        self.map_data, self.map_res = None, 0.06
        self.map_width, self.map_height = 0, 0
        self.map_origin_x, self.map_origin_y = 0.0, 0.0
        
        # LOWERED TO 35: Restricts candidates strictly to open space, far from wall inflation
        self.MAX_SAFE_COST = 35
        self.last_target_x, self.last_target_y = None, None
        self.MIN_UPDATE_DIST = 0.03
        self.memory_bank: List[Tuple[float, float, float]] = []
        self.memory_space = 3

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Evasion Calculator Initialized (Open-Space Priority Enabled).")

    def state_cb(self, msg):
        if self.current_state != msg.data and msg.data != 'EVADING':
            self.memory_bank.clear()
            self.last_target_x = None
        self.current_state = msg.data

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def costmap_callback(self, msg: OccupancyGrid):
        self.map_data = np.array(msg.data, dtype=np.int16)
        self.map_res = msg.info.resolution
        self.map_width, self.map_height = msg.info.width, msg.info.height
        self.map_origin_x, self.map_origin_y = msg.info.origin.position.x, msg.info.origin.position.y

    def get_cell_cost(self, x: float, y: float) -> int:
        if self.map_data is None: 
            return 255
        raw_col, raw_row = int((x - self.map_origin_x) / self.map_res), int((y - self.map_origin_y) / self.map_res)
        col = max(0, min(raw_col, self.map_width - 1))
        row = max(0, min(raw_row, self.map_height - 1))

        MARGIN = 8
        if raw_col < MARGIN or raw_col >= (self.map_width - MARGIN) or \
           raw_row < MARGIN or raw_row >= (self.map_height - MARGIN):
            return 255
        cost = self.map_data[row * self.map_width + col]
        return 255 if cost == -1 else cost

    def is_path_clear(self, start_x: float, start_y: float, end_x: float, end_y: float) -> bool:
        if self.map_data is None: 
            return False
        dist = math.hypot(end_x - start_x, end_y - start_y)
        # Fine step size (0.03m) guarantees every costmap cell is checked
        steps = max(3, int(dist / 0.03))
        t = np.linspace(0.0, 1.0, steps)
        px, py = start_x + t * (end_x - start_x), start_y + t * (end_y - start_y)

        cols = np.clip(((px - self.map_origin_x) / self.map_res).astype(np.int32), 0, self.map_width - 1)
        rows = np.clip(((py - self.map_origin_y) / self.map_res).astype(np.int32), 0, self.map_height - 1)
        costs = self.map_data[rows * self.map_width + cols]

        if np.any(costs > self.MAX_SAFE_COST) or np.any(costs < 0): 
            return False
        return True

    def is_path_clear_dogleg(self, start_x, start_y, end_x, end_y):
        if self.is_path_clear(start_x, start_y, end_x, end_y):
            return True, None

        direct_angle = math.atan2(end_y - start_y, end_x - start_x)
        mid_dist = math.hypot(end_x - start_x, end_y - start_y) / 2.0
        
        test_angles = [math.radians(35), math.radians(-35), math.radians(50), math.radians(-50)]
        for offset in test_angles:
            mid_x = start_x + (mid_dist * math.cos(direct_angle + offset))
            mid_y = start_y + (mid_dist * math.sin(direct_angle + offset))
            if self.is_path_clear(start_x, start_y, mid_x, mid_y):
                if self.is_path_clear(mid_x, mid_y, end_x, end_y):
                    return True, (mid_x, mid_y)
        return False, None

    def check_memory_bank(self, our_x, our_y, enemy_x, enemy_y):
        if not self.memory_bank: 
            return []
        still_valid_points = []
        current_dist_to_enemy = math.hypot(our_x - enemy_x, our_y - enemy_y)
        
        for mx, my, myaw in self.memory_bank:
            if math.hypot(mx - enemy_x, my - enemy_y) <= (current_dist_to_enemy + 0.2): 
                continue
            is_clear, via_point = self.is_path_clear_dogleg(our_x, our_y, mx, my)
            if not is_clear: 
                continue
            if self.get_cell_cost(mx, my) > self.MAX_SAFE_COST: 
                continue
            still_valid_points.append((mx, my, myaw, via_point))
        return still_valid_points

    def get_adaptive_weights(self, current_cost, dist_to_enemy):
        w_dist, w_speed, w_turn, w_cost = 8.0, 4.0, 6.0, 2.0  
        if current_cost > 15: 
            w_cost, w_turn = 5.0, 8.0  
        if dist_to_enemy < 1.2: 
            w_dist, w_speed = 18.0, 5.0
        return {'w_dist': w_dist, 'w_speed': w_speed, 'w_turn': w_turn, 'w_cost': w_cost}

    def evaluate_future_options(self, cand_x, cand_y):
        future_clear_branches = 0
        probe_radius = self.sample_radii[0]

        for cos_val, sin_val in self.angle_steps[::3]:
            next_x, next_y = cand_x + (probe_radius * cos_val), cand_y + (probe_radius * sin_val)
            is_clear, _ = self.is_path_clear_dogleg(cand_x, cand_y, next_x, next_y)
            if is_clear and self.get_cell_cost(next_x, next_y) <= self.MAX_SAFE_COST:
                future_clear_branches += 1
        return future_clear_branches * 1.0

    def get_safe_escape_points(self, robot_x, robot_y):
        safe_points = []
        for r in self.sample_radii:
            for cos_val, sin_val in self.angle_steps:
                cand_x, cand_y = robot_x + (r * cos_val), robot_y + (r * sin_val)
                cost = self.get_cell_cost(cand_x, cand_y)
                if cost > self.MAX_SAFE_COST:
                    continue
                is_clear, via_point = self.is_path_clear_dogleg(robot_x, robot_y, cand_x, cand_y)
                if is_clear:
                    safe_points.append((cand_x, cand_y, r, cost, via_point))
        return safe_points

    def get_best_escape_target(self, safe_points, our_x, our_y, our_yaw, enemy_x, enemy_y):
        best_point, highest_score, all_scored_points = None, -float('inf'), []
        weights = self.get_adaptive_weights(self.get_cell_cost(our_x, our_y), math.hypot(our_x - enemy_x, our_y - enemy_y))

        for cand_x, cand_y, r, cost, via_point in safe_points:
            angle_to_cand = math.atan2(cand_y - our_y, cand_x - our_x)
            turn_diff = abs(math.atan2(math.sin(angle_to_cand - our_yaw), math.cos(angle_to_cand - our_yaw)))
            
            # Discard sharp turns (greater than 65 deg) to eliminate spinning in place
            if turn_diff > math.radians(65): 
                continue
                
            score = (math.hypot(cand_x - enemy_x, cand_y - enemy_y) * weights['w_dist']) + (r * weights['w_speed'])
            score -= (turn_diff * weights['w_turn']) + (cost * weights['w_cost'])
            
            # Substantial bonus for ultra-clean open space (cost < 10)
            if 0 <= cost < 10: 
                score += 15.0

            score += self.evaluate_future_options(cand_x, cand_y)
            all_scored_points.append((score, cand_x, cand_y, angle_to_cand))

            if score > highest_score: 
                highest_score, best_point = score, (cand_x, cand_y, angle_to_cand, via_point)

        all_scored_points.sort(key=lambda item: item[0], reverse=True)
        self.memory_bank = [(item[1], item[2], item[3]) for item in all_scored_points[:self.memory_space]]
        return best_point

    def control_loop(self):
        if self.map_data is None: 
            return
        if self.current_state != 'EVADING':
            self.last_target_x = None
            self.memory_bank.clear()
            return

        try:
            tf_global = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
            tf_en = self.tf_buffer.lookup_transform(self.map_frame, self.enemy_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return

        global_x, global_y = tf_global.transform.translation.x, tf_global.transform.translation.y
        our_yaw = self.get_yaw_from_quaternion(tf_global.transform.rotation)
        en_x, en_y = tf_en.transform.translation.x, tf_en.transform.translation.y

        via_point = None
        valid_cached_points = self.check_memory_bank(global_x, global_y, en_x, en_y)
        if valid_cached_points:
            target_x, target_y, target_yaw, via_point = valid_cached_points[0]
        else:
            safe_points = self.get_safe_escape_points(global_x, global_y)
            if not safe_points:
                dx, dy = global_x - en_x, global_y - en_y
                dist = max(0.01, math.hypot(dx, dy))
                target_x, target_y = global_x + (dx / dist) * 0.8, global_y + (dy / dist) * 0.8
                target_yaw = math.atan2(dy, dx)
                via_point = None
            else:
                best_target = self.get_best_escape_target(safe_points, global_x, global_y, our_yaw, en_x, en_y)
                if best_target is None: 
                    return
                target_x, target_y, target_yaw, via_point = best_target
        
        if self.last_target_x is not None:
            if math.hypot(target_x - self.last_target_x, target_y - self.last_target_y) < self.MIN_UPDATE_DIST:
                target_x, target_y, target_yaw = self.last_target_x, self.last_target_y, self.last_target_yaw

        self.last_target_x, self.last_target_y, self.last_target_yaw = target_x, target_y, target_yaw

        # Broadcast Main Goal
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.target_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = target_x, target_y, 0.0
        t.transform.rotation.z, t.transform.rotation.w = math.sin(target_yaw / 2.0), math.cos(target_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Broadcast Via-Point
        t_via = TransformStamped()
        t_via.header.stamp = self.get_clock().now().to_msg()
        t_via.header.frame_id = self.map_frame
        t_via.child_frame_id = self.via_frame
        t_via.transform.translation.x = via_point[0] if via_point else target_x
        t_via.transform.translation.y = via_point[1] if via_point else target_y
        t_via.transform.rotation = t.transform.rotation
        self.tf_broadcaster.sendTransform(t_via)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EvasionCalculator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
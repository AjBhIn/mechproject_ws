#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

class EvasionCalculator(Node):
    def __init__(self):
        super().__init__('evasion_calculator')

        # --- Parameters ---
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'our_bot/base_link')
        self.declare_parameter('enemy_frame', 'enemy_bot/base_link')
        self.declare_parameter('target_frame', 'dynamic_nav_target') # The frame it will broadcast
        self.declare_parameter('costmap_topic', 'our_bot/global_costmap/costmap')

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.enemy_frame = self.get_parameter('enemy_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        
        # --- TF & Subscriptions ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.get_parameter('costmap_topic').value, self.costmap_cb, 10
        )

        # --- Precomputations for Speed ---
        self.sample_radii = [0.8, 1.2, 1.8, 2.0]
        # Precompute 360 degrees into 8-degree slices (Cos, Sin)
        self.angle_steps = [(math.cos(math.radians(d)), math.sin(math.radians(d))) for d in range(0, 360, 8)]
        
        self.map_data, self.map_res = None, 0.05
        self.map_width = self.map_height = 0
        self.map_origin_x = self.map_origin_y = 0.0
        self.MAX_SAFE_COST = 100
        
        self.last_target_x = self.last_target_y = None
        self.MIN_UPDATE_DIST = 0.20
        self.memory_bank = []

        self.timer = self.create_timer(0.1, self.control_loop)

    def costmap_cb(self, msg: OccupancyGrid):
        """Flattens the costmap array into a fast NumPy array for raycasting."""
        self.map_data = np.array(msg.data, dtype=np.int16)
        self.map_res = msg.info.resolution
        self.map_width, self.map_height = msg.info.width, msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def get_yaw_from_q(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def evaluate_threat(self, rel_x, rel_y):
        """Determines if the enemy is in our danger zone."""
        distance = math.hypot(rel_x, rel_y)
        angle_deg = abs(math.degrees(math.atan2(rel_y, rel_x)))
        
        # 67.5deg Front Cone = 2.8m danger | Rear Cone = 0.0m (Safe) | Flanks = 2.0m
        threshold = 2.8 if angle_deg <= 67.5 else (0.0 if angle_deg >= 135.0 else 2.0)
        
        if distance <= threshold:
            return True, distance, math.atan2(rel_y, rel_x)
        return False, distance, None

    def get_cell_cost(self, x, y):
        """Safely fetches costmap data without throwing index errors."""
        if self.map_data is None: return 255
        col = max(0, min(int((x - self.map_origin_x) / self.map_res), self.map_width - 1))
        row = max(0, min(int((y - self.map_origin_y) / self.map_res), self.map_height - 1))
        cost = self.map_data[row * self.map_width + col]
        return 255 if cost == -1 else cost

    def is_path_clear(self, sx, sy, ex, ey):
        """Draws a mathematical line between two points and checks if it hits a wall."""
        if self.map_data is None: return False
        dist = math.hypot(ex - sx, ey - sy)
        steps = max(2, int(dist / (0.10 if dist <= 0.8 else 0.05)))
        
        t = np.linspace(0.0, 1.0, steps)
        px, py = sx + t * (ex - sx), sy + t * (ey - sy)
        
        cols = np.clip(((px - self.map_origin_x) / self.map_res).astype(np.int32), 0, self.map_width - 1)
        rows = np.clip(((py - self.map_origin_y) / self.map_res).astype(np.int32), 0, self.map_height - 1)
        costs = self.map_data[rows * self.map_width + cols]
        
        return not (np.any(costs > self.MAX_SAFE_COST) or np.any(costs == -1))

    def check_memory_bank(self, ox, oy, ex, ey):
        """Checks if a previously calculated escape point is still safe (saves CPU time)."""
        valid = []
        cur_dist = math.hypot(ox - ex, oy - ey)
        for mx, my, myaw in self.memory_bank:
            if math.hypot(mx - ex, my - ey) > (cur_dist + 0.3) and self.is_path_clear(ox, oy, mx, my) and self.get_cell_cost(mx, my) <= self.MAX_SAFE_COST:
                valid.append((mx, my, myaw))
        return valid

    def get_adaptive_weights(self, cost, dist):
        """Changes how escape points are scored based on how close the enemy is."""
        return {
            'w_dist': 20.0 if dist < 1.2 else 10.0,
            'w_speed': 4.0 if dist < 1.2 else 2.0,
            'w_turn': 0.5 if cost > 30 else 4.0, # Turn sharper if near walls
            'w_cost': 1.0 if cost > 30 else 0.3
        }

    def evaluate_future_options(self, cx, cy):
        """Looks 1 step ahead of candidate points to prevent driving into dead-ends."""
        branches = 0
        for cos_v, sin_v in self.angle_steps[::4]:
            nx, ny = cx + (0.5 * cos_v), cy + (0.5 * sin_v)
            if self.is_path_clear(cx, cy, nx, ny) and self.get_cell_cost(nx, ny) <= self.MAX_SAFE_COST:
                branches += 1
        return branches * 1.5

    def get_best_escape(self, safe_points, ox, oy, oyaw, ex, ey):
        """Scores all safe points and returns the absolute best one."""
        best_pt, max_score, scored = None, -float('inf'), []
        weights = self.get_adaptive_weights(self.get_cell_cost(ox, oy), math.hypot(ox - ex, oy - ey))

        for cx, cy, r, cost in safe_points:
            angle = math.atan2(cy - oy, cx - ox)
            
            # Master Scoring Formula
            score = (math.hypot(cx - ex, cy - ey) * weights['w_dist'] +
                     r * weights['w_speed'] -
                     abs(math.atan2(math.sin(angle - oyaw), math.cos(angle - oyaw))) * weights['w_turn'] -
                     cost * weights['w_cost'] +
                     (12.0 if 0 <= cost < 15 else 0.0) +
                     self.evaluate_future_options(cx, cy))
            
            scored.append((score, cx, cy, angle))
            if score > max_score:
                max_score, best_pt = score, (cx, cy, angle)

        # Save top 3 points for the next loop
        scored.sort(key=lambda x: x[0], reverse=True)
        self.memory_bank = [(i[1], i[2], i[3]) for i in scored[:3]]
        return best_pt

    def control_loop(self):
        """Main Loop: Measures threat, calculates escape, broadcasts TF."""
        try:
            tf_glob = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            tf_en = self.tf_buffer.lookup_transform(self.map_frame, self.enemy_frame, rclpy.time.Time())
            tf_rel = self.tf_buffer.lookup_transform(self.enemy_frame, self.robot_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return

        gx, gy = tf_glob.transform.translation.x, tf_glob.transform.translation.y
        gyaw = self.get_yaw_from_q(tf_glob.transform.rotation)
        ex, ey = tf_en.transform.translation.x, tf_en.transform.translation.y

        # 1. Are we in danger?
        in_danger, _, _ = self.evaluate_threat(tf_rel.transform.translation.x, tf_rel.transform.translation.y)
        
        if not in_danger:
            # We are safe. Do not calculate paths. Do not broadcast TF.
            self.last_target_x = None
            self.memory_bank.clear()
            return 

        # 2. We are in danger. Calculate route.
        cached = self.check_memory_bank(gx, gy, ex, ey)
        if cached:
            tx, ty, tyaw = cached[0]
        else:
            # Generate radial points, filter by raycast, and score them
            safe_pts = [(gx + r * c, gy + r * s, r, self.get_cell_cost(gx + r * c, gy + r * s)) 
                        for r in self.sample_radii for c, s in self.angle_steps 
                        if self.is_path_clear(gx, gy, gx + r * c, gy + r * s)]
            
            if not safe_pts: return
            best = self.get_best_escape(safe_pts, gx, gy, gyaw, ex, ey)
            if not best: return
            tx, ty, tyaw = best
        
        # Deadband filter to prevent jitter
        if self.last_target_x and math.hypot(tx - self.last_target_x, ty - self.last_target_y) < self.MIN_UPDATE_DIST:
            return  

        self.last_target_x, self.last_target_y = tx, ty

        # 3. Broadcast the chosen escape point to TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.target_frame
        t.transform.translation.x, t.transform.translation.y = tx, ty
        t.transform.rotation.z, t.transform.rotation.w = math.sin(tyaw / 2.0), math.cos(tyaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EvasionCalculator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
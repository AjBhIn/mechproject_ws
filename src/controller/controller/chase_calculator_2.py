#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from geometry_msgs.msg import TransformStamped

class ChaserCalculator(Node):
    def __init__(self):
        super().__init__('chaser_calculator')

        self.map_frame = 'map'
        self.pursuer_frame = 'our_bot/base_link'
        self.carrot_frame = 'enemy_bot/target_point'
        self.broadcast_frame = 'chase_nav_target'

        self.short_goal_dist = 0.8
        self.current_state = 'CHASING'
        self.costmap = None

        # Predictive Velocity Tracking
        self.last_enemy_x = None
        self.last_enemy_y = None
        self.last_enemy_time = None
        self.enemy_vx = 0.0
        self.enemy_vy = 0.0

        # OccupancyGrid threshold (0 to 100)
        self.MAX_SAFE_COST = 35

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(String, 'our_bot/robot_state', self.state_cb, 10)
        self.create_subscription(OccupancyGrid, '/our_bot/global_costmap/costmap', self.costmap_cb, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Chaser Calculator Initialized (Predictive Lead Enabled).")

    def state_cb(self, msg):
        self.current_state = msg.data

    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    def is_pose_safe(self, x: float, y: float) -> bool:
        if self.costmap is None: 
            return True
        info = self.costmap.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        if 0 <= gx < info.width and 0 <= gy < info.height:
            cost = self.costmap.data[gy * info.width + gx]
            return 0 <= cost < self.MAX_SAFE_COST
        return False

    def project_to_free_space(self, start_x: float, start_y: float, target_x: float, target_y: float):
        if self.is_pose_safe(target_x, target_y): 
            return target_x, target_y
        if math.hypot(target_x - start_x, target_y - start_y) < 0.05: 
            return start_x, start_y
        for i in range(1, 10):
            alpha = 1.0 - (i / 10.0)
            px = start_x + alpha * (target_x - start_x)
            py = start_y + alpha * (target_y - start_y)
            if self.is_pose_safe(px, py): 
                return px, py
        return start_x, start_y

    def update_enemy_velocity(self, c_x: float, c_y: float, now_sec: float):
        if self.last_enemy_time is not None:
            dt = now_sec - self.last_enemy_time
            if dt > 0.01:
                vx_raw = (c_x - self.last_enemy_x) / dt
                vy_raw = (c_y - self.last_enemy_y) / dt
                alpha = 0.3
                self.enemy_vx = alpha * vx_raw + (1.0 - alpha) * self.enemy_vx
                self.enemy_vy = alpha * vy_raw + (1.0 - alpha) * self.enemy_vy
        
        self.last_enemy_x = c_x
        self.last_enemy_y = c_y
        self.last_enemy_time = now_sec

    def control_loop(self):
        if self.current_state != 'CHASING':
            return

        try:
            t_our = self.tf_buffer.lookup_transform(self.map_frame, self.pursuer_frame, rclpy.time.Time())
            t_carrot = self.tf_buffer.lookup_transform(self.map_frame, self.carrot_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return

        p_x, p_y = t_our.transform.translation.x, t_our.transform.translation.y
        c_x, c_y = t_carrot.transform.translation.x, t_carrot.transform.translation.y
        c_rot = t_carrot.transform.rotation

        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.update_enemy_velocity(c_x, c_y, now_sec)

        dist_to_carrot = math.hypot(c_x - p_x, c_y - p_y)
        estimated_our_speed = 0.4  
        lead_time = min(1.0, dist_to_carrot / estimated_our_speed)  

        pred_c_x = c_x + (self.enemy_vx * lead_time)
        pred_c_y = c_y + (self.enemy_vy * lead_time)

        dx = pred_c_x - p_x
        dy = pred_c_y - p_y
        dist_pred = math.hypot(dx, dy)

        if dist_pred > self.short_goal_dist:
            ratio = self.short_goal_dist / dist_pred
            target_x = p_x + dx * ratio
            target_y = p_y + dy * ratio
        else:
            target_x = pred_c_x
            target_y = pred_c_y

        safe_x, safe_y = self.project_to_free_space(p_x, p_y, target_x, target_y)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.broadcast_frame
        t.transform.translation.x = safe_x
        t.transform.translation.y = safe_y
        t.transform.rotation = c_rot

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ChaserCalculator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
import tf2_ros

class EvasionCalculator(Node):
    def __init__(self):
        super().__init__('evasion_calculator')

        self.current_state = 'CHASING'
        self.costmap = None

        # TF Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(String, 'our_bot/robot_state', self.state_callback, 10)
        self.create_subscription(OccupancyGrid, 'our_bot/global_costmap/costmap', self.costmap_callback, 10)

        # Run at 10 Hz
        self.create_timer(0.1, self.compute_escape_point)

    def state_callback(self, msg):
        self.current_state = msg.data

    def costmap_callback(self, msg):
        self.costmap = msg

    def is_safe(self, x, y):
        """Returns True if coordinate (x, y) is clear of obstacles."""
        if not self.costmap:
            return True
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        res = self.costmap.info.resolution

        grid_x = int((x - origin_x) / res)
        grid_y = int((y - origin_y) / res)

        if 0 <= grid_x < self.costmap.info.width and 0 <= grid_y < self.costmap.info.height:
            cost = self.costmap.data[grid_y * self.costmap.info.width + grid_x]
            return (cost >= 0 and cost < 100)
        return False

    def compute_escape_point(self):
        # Calculate only when actively evading
        if self.current_state != 'EVADING':
            return

        try:
            our_tf = self.tf_buffer.lookup_transform('map', 'our_bot/base_link', rclpy.time.Time())
            enemy_tf = self.tf_buffer.lookup_transform('map', 'enemy_bot/base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return

        our_x = our_tf.transform.translation.x
        our_y = our_tf.transform.translation.y
        enemy_x = enemy_tf.transform.translation.x
        enemy_y = enemy_tf.transform.translation.y

        best_x = our_x
        best_y = our_y
        best_distance = -1.0

        # Sample points in circles around our robot (1.0m, 1.5m, 2.0m)
        radii = [1.0, 1.5, 2.0]
        angles = [0, 45, 90, 135, 180, 225, 270, 315]

        for r in radii:
            for deg in angles:
                rad = math.radians(deg)
                cand_x = our_x + r * math.cos(rad)
                cand_y = our_y + r * math.sin(rad)

                # Check if candidate point is clear of walls
                if self.is_safe(cand_x, cand_y):
                    dist_to_enemy = math.hypot(cand_x - enemy_x, cand_y - enemy_y)
                    if dist_to_enemy > best_distance:
                        best_distance = dist_to_enemy
                        best_x = cand_x
                        best_y = cand_y

        # Broadcast best point as dynamic_nav_target frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'dynamic_nav_target'
        t.transform.translation.x = best_x
        t.transform.translation.y = best_y
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EvasionCalculator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TargetBroadcaster(Node):
    def __init__(self):
        super().__init__('target_broadcaster')
        
        # --- Parameters ---
        self.declare_parameter('parent_frame', 'enemy_bot/base_link')
        self.declare_parameter('child_frame', 'enemy_bot/target_point')
        self.declare_parameter('distance_behind', 1.0) # Meters to project behind the enemy

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.distance = self.get_parameter('distance_behind').value

        # --- TF Setup ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Broadcast at roughly 30Hz
        self.timer = self.create_timer(0.033, self.broadcast_frame)

    def broadcast_frame(self):
        """Creates and broadcasts the 'carrot' TF frame behind the enemy."""
        t = TransformStamped()
        
        # Lock to simulation time
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        
        # Push the coordinate backwards along the X-axis of the enemy
        t.transform.translation.x = -float(self.distance)
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TargetBroadcaster())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
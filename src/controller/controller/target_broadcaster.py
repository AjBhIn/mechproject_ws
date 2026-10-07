#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TargetBroadcaster(Node):
    def __init__(self):
        super().__init__('target_broadcaster')

        self.parent_frame = 'enemy_bot/base_link'
        self.child_frame = 'enemy_bot/target_point'
        self.distance_behind = 1.0  # 1 meter behind enemy

        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publish at 30 Hz
        self.create_timer(0.033, self.publish_tf)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Offset 1.0 meter backward along the X axis
        t.transform.translation.x = -float(self.distance_behind)
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TargetBroadcaster())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
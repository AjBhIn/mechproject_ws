#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')

        self.current_state = 'CHASING'
        self.last_sent_x = None
        self.last_sent_y = None

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'our_bot/navigate_to_pose')

        # Subscriptions
        self.create_subscription(String, 'our_bot/robot_state', self.state_callback, 10)

        # Check loop at 10 Hz
        self.create_timer(0.1, self.control_loop)

    def state_callback(self, msg):
        if self.current_state != msg.data:
            self.current_state = msg.data
            self.last_sent_x = None
            self.last_sent_y = None

    def control_loop(self):
        if not self.nav_client.wait_for_server(timeout_sec=0.02):
            return

        # Select target frame based on active state
        if self.current_state == 'CHASING':
            target_frame = 'enemy_bot/target_point'
        else:
            target_frame = 'dynamic_nav_target'

        try:
            target_tf = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
            target_x = target_tf.transform.translation.x
            target_y = target_tf.transform.translation.y
            target_rot = target_tf.transform.rotation
        except tf2_ros.TransformException:
            return

        # Only send goal if target moved more than 0.15m to avoid action spamming
        if self.last_sent_x is not None:
            moved = math.hypot(target_x - self.last_sent_x, target_y - self.last_sent_y)
            if moved < 0.15:
                return

        # Send goal to Nav2
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_x
        goal_msg.pose.position.y = target_y
        goal_msg.pose.orientation = target_rot

        self.nav_client.send_goal_async(goal_msg)

        self.last_sent_x = target_x
        self.last_sent_y = target_y

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Nav2GoalSender())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
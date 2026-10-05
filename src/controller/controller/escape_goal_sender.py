#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros

class EscapeGoalSender(Node):
    def __init__(self):
        super().__init__('escape_goal_sender')

        # --- Parameters ---
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('action_name', 'our_bot/navigate_to_pose')
        self.declare_parameter('escape_target_frame', 'dynamic_nav_target')

        self.map_frame = self.get_parameter('map_frame').value
        self.action_name = self.get_parameter('action_name').value
        self.escape_tf = self.get_parameter('escape_target_frame').value

        # --- ROS Interfaces ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)

        self.is_goal_pending = False
        
        # Check for the TF frame at 1Hz
        self.timer = self.create_timer(1.0, self.control_loop)

    def get_escape_pose(self):
        """Looks for the dynamic_nav_target TF. Returns None if it doesn't exist."""
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.escape_tf, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.orientation = tf.transform.rotation
            return pose
            
        except tf2_ros.TransformException:
            # The Evasion Calculator is not broadcasting. We are safe.
            return None

    def control_loop(self):
        """Sends the escape coordinate to Nav2."""
        target_pose = self.get_escape_pose()
        
        # If no target exists, or we are waiting on a previous goal, do nothing.
        if not target_pose or self.is_goal_pending: return

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            return

        # Fire the goal to Nav2!
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        self.is_goal_pending = True

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_cb)

    def goal_cb(self, future):
        """Resets the lock once Nav2 accepts the goal."""
        self.is_goal_pending = False
        if future.result().accepted:
            self.get_logger().info("Escape coordinate accepted by Nav2!")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EscapeGoalSender())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
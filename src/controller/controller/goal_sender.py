#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')

        self.map_frame = 'map'
        self.action_name = 'our_bot/navigate_to_pose'

        self.chase_frame = 'chase_nav_target'
        self.escape_frame = 'dynamic_nav_target'

        self.min_dist_threshold = 0.15
        self.state = 'CHASING'
        self.last_sent_pose = None
        self.active_goal_handle = None
        self.is_goal_pending = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state_sub = self.create_subscription(String, 'our_bot/robot_state', self.state_cb, 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Unified Goal Sender Initialized.")

    def state_cb(self, msg):
        new_state = msg.data
        if self.state != new_state:
            self.state = new_state
            self.cancel_active_goal()
            self.last_sent_pose = None

    def cancel_active_goal(self):
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

    def get_target_tf(self):
        # Decide which TF to follow based on state
        target_frame = self.escape_frame if self.state == 'EVADING' else self.chase_frame
        try:
            trans = self.tf_buffer.lookup_transform(self.map_frame, target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.03))
            return trans
        except tf2_ros.TransformException:
            return None

    def goal_response_callback(self, future):
        self.is_goal_pending = False
        goal_handle = future.result()
        if goal_handle.accepted:
            self.active_goal_handle = goal_handle

    def control_loop(self):
        if not self.nav_client.wait_for_server(timeout_sec=0.02): return
        if self.is_goal_pending: return

        target_tf = self.get_target_tf()
        if target_tf is None: return

        t_x = target_tf.transform.translation.x
        t_y = target_tf.transform.translation.y

        # Deadband resend threshold logic
        if self.last_sent_pose is not None:
            dx = t_x - self.last_sent_pose.pose.position.x
            dy = t_y - self.last_sent_pose.pose.position.y
            if math.hypot(dx, dy) < self.min_dist_threshold: return

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = t_x
        goal.pose.position.y = t_y
        goal.pose.orientation = target_tf.transform.rotation

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self.is_goal_pending = True
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)
        self.last_sent_pose = goal

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GoalSender())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
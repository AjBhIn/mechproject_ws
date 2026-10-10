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

        self.last_goal_time = 0.0
        self.state = 'CHASING'
        self.last_sent_pose = None
        self.active_goal_handle = None
        self.is_goal_pending = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state_sub = self.create_subscription(String, 'our_bot/robot_state', self.state_cb, 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Robust Goal Sender initialized with ADAPTIVE pacing.")

    def state_cb(self, msg):
        new_state = msg.data
        if self.state != new_state:
            self.get_logger().info(f"State transition: {self.state} -> {new_state}. Canceling active goals.")
            self.state = new_state
            self.cancel_active_goal()
            
            # --- ADD THIS LINE TO PREVENT THE FREEZE ---
            self.is_goal_pending = False  
            
            self.last_sent_pose = None
            self.last_goal_time = 0.0

    def cancel_active_goal(self):
        if self.active_goal_handle is not None and self.active_goal_handle.accepted:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

    def get_target_tf(self):
        target_frame = self.escape_frame if self.state == 'EVADING' else self.chase_frame
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, target_frame, rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            return trans
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            return None

    def goal_response_callback(self, future):
        self.is_goal_pending = False
        goal_handle = future.result()
        if goal_handle.accepted:
            self.active_goal_handle = goal_handle
        else:
            self.active_goal_handle = None

    def control_loop(self):
        if not self.nav_client.wait_for_server(timeout_sec=0.01): 
            return
        if self.is_goal_pending: 
            return

        target_tf = self.get_target_tf()
        if target_tf is None: 
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        t_x = target_tf.transform.translation.x
        t_y = target_tf.transform.translation.y

        try:
            robot_tf = self.tf_buffer.lookup_transform(self.map_frame, 'our_bot/base_link', rclpy.time.Time())
            dist_to_target = math.hypot(t_x - robot_tf.transform.translation.x, t_y - robot_tf.transform.translation.y)
        except:
            dist_to_target = 2.0

        # TUNING POINT: ADAPTIVE PACING THRESHOLDS
        # If your physical bot suffers from heavy stop-and-go stuttering, increase dynamic_min_time (lower the Hz).
        # If your bot tracks too loosely, decrease dynamic_min_time (higher Hz) and decrease dynamic_min_dist.
        if dist_to_target < 1.5:
            dynamic_min_dist = 0.10   # Only update if target moves 10cm
            dynamic_min_time = 0.25   # Max 4 Hz goal sending
        else:
            dynamic_min_dist = 0.30   # Only update if target moves 30cm
            dynamic_min_time = 0.50   # Max 2 Hz goal sending

        if self.last_sent_pose is not None:
            dx = t_x - self.last_sent_pose.pose.position.x
            dy = t_y - self.last_sent_pose.pose.position.y
            dist_moved = math.hypot(dx, dy)
            time_elapsed = now_sec - self.last_goal_time

            if dist_moved < dynamic_min_dist or time_elapsed < dynamic_min_time:
                return

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = t_x
        goal.pose.position.y = t_y
        goal.pose.orientation = target_tf.transform.rotation

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self.cancel_active_goal()

        self.is_goal_pending = True
        self.last_goal_time = now_sec
        self.last_sent_pose = goal

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GoalSender())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
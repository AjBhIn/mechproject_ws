#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient
import tf2_ros

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')

        self.map_frame = 'map'
        self.chase_frame = 'chase_nav_target'
        self.escape_frame = 'dynamic_nav_target'
        self.escape_via_frame = 'dynamic_nav_target_via'

        self.last_goal_time = 0.0
        self.state = 'CHASING'
        self.last_sent_pose = None
        self.active_goal_handle = None
        self.is_goal_pending = False
        self.force_update = False 

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state_sub = self.create_subscription(String, 'our_bot/robot_state', self.state_cb, 10)
        
        self.nav_pose_client = ActionClient(self, NavigateToPose, 'our_bot/navigate_to_pose')
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, 'our_bot/navigate_through_poses')
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Goal Sender Initialized (Multi-Pose Dogleg Compliant).")

    def state_cb(self, msg):
        new_state = msg.data
        if self.state != new_state:
            self.get_logger().info(f"State transition: {self.state} -> {new_state}. Preempting active goal.")
            self.state = new_state
            self.force_update = True 
            if self.active_goal_handle is not None:
                self.active_goal_handle.cancel_goal_async()
                self.is_goal_pending = False

    def get_target_tf(self, frame):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, frame, rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except tf2_ros.TransformException:
            return None

    def create_pose_msg(self, trans):
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = trans.transform.translation.x
        goal.pose.position.y = trans.transform.translation.y
        goal.pose.orientation = trans.transform.rotation
        return goal

    def goal_response_callback(self, future):
        self.is_goal_pending = False
        goal_handle = future.result()
        if goal_handle.accepted:
            self.active_goal_handle = goal_handle
        else:
            self.active_goal_handle = None

    def control_loop(self):
        if self.is_goal_pending: 
            return

        target_tf = self.get_target_tf(self.escape_frame if self.state == 'EVADING' else self.chase_frame)
        if target_tf is None: 
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        t_x, t_y = target_tf.transform.translation.x, target_tf.transform.translation.y

        try:
            robot_tf = self.tf_buffer.lookup_transform(self.map_frame, 'our_bot/base_link', rclpy.time.Time())
            dist_to_target = math.hypot(t_x - robot_tf.transform.translation.x, t_y - robot_tf.transform.translation.y)
        except tf2_ros.TransformException:
            dist_to_target = 2.0

        dynamic_min_dist, dynamic_min_time = (0.10, 0.15) if dist_to_target < 1.5 else (0.30, 0.50)

        if not self.force_update and self.last_sent_pose is not None:
            dist_moved = math.hypot(t_x - self.last_sent_pose.pose.position.x, t_y - self.last_sent_pose.pose.position.y)
            if dist_moved < dynamic_min_dist or (now_sec - self.last_goal_time) < dynamic_min_time:
                return

        self.force_update = False
        final_goal_msg = self.create_pose_msg(target_tf)

        if self.state == 'EVADING':
            via_tf = self.get_target_tf(self.escape_via_frame)
            if via_tf is not None:
                via_dist = math.hypot(via_tf.transform.translation.x - t_x, via_tf.transform.translation.y - t_y)
                
                if via_dist > 0.1:
                    if not self.nav_through_client.wait_for_server(timeout_sec=0.01): 
                        return
                    via_goal_msg = self.create_pose_msg(via_tf)
                    
                    goal = NavigateThroughPoses.Goal()
                    goal.poses = [via_goal_msg, final_goal_msg]
                    
                    self.is_goal_pending = True
                    self.nav_through_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)
                    self.last_goal_time, self.last_sent_pose = now_sec, final_goal_msg
                    return

        if not self.nav_pose_client.wait_for_server(timeout_sec=0.01): 
            return
        
        goal = NavigateToPose.Goal()
        goal.pose = final_goal_msg
        
        self.is_goal_pending = True
        self.nav_pose_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)
        self.last_goal_time, self.last_sent_pose = now_sec, final_goal_msg

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GoalSender())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
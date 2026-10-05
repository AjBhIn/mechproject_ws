#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import tf2_ros

class ChaserController(Node):
    def __init__(self):
        super().__init__('chaser_controller')
        self.cb_group = ReentrantCallbackGroup()

        # --- Parameters ---
        params = {
            'map_frame': 'map', 'pursuer_frame': 'our_bot/base_link', 'enemy_frame': 'enemy_bot/base_link',
            'target_frame': 'enemy_bot/target_point', 'costmap_topic': 'our_bot/global_costmap/costmap',
            'action_name': 'our_bot/navigate_to_pose', 'cmd_vel_topic': 'our_bot/cmd_vel',
            'short_goal_dist': 0.8, 'evade_trigger_dist': 2.0, 'evade_clear_dist': 2.8, 'min_dist_threshold': 0.25
        }
        for k, v in params.items(): self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in params}

        # --- State Tracking ---
        self.state = 'CHASING'
        self.last_sent_pose = None
        self.active_goal = None
        self.costmap = None

        # --- ROS Interfaces ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.create_subscription(OccupancyGrid, self.p['costmap_topic'], self.costmap_cb, 10, callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.p['action_name'], callback_group=self.cb_group)
        
        # 10Hz control loop
        self.create_timer(0.1, self.control_loop, callback_group=self.cb_group)

    def costmap_cb(self, msg): 
        self.costmap = msg

    def is_pose_safe(self, x, y):
        """Checks if a coordinate sits on a lethal obstacle."""
        if not self.costmap: return True
        gx = int((x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
        gy = int((y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
        if 0 <= gx < self.costmap.info.width and 0 <= gy < self.costmap.info.height:
            return self.costmap.data[gy * self.costmap.info.width + gx] < 180
        return False

    def project_to_free_space(self, sx, sy, tx, ty):
        """Pulls a goal back towards the robot if it accidentally touches a wall."""
        if self.is_pose_safe(tx, ty): return tx, ty
        if math.hypot(tx - sx, ty - sy) < 0.05: return sx, sy
        for i in range(1, 10):
            px, py = sx + (1.0 - (i / 10.0)) * (tx - sx), sy + (1.0 - (i / 10.0)) * (ty - sy)
            if self.is_pose_safe(px, py): return px, py
        return sx, sy

    def get_pose(self, frame):
        """Helper to get X, Y, Yaw from TF."""
        try:
            t = self.tf_buffer.lookup_transform(self.p['map_frame'], frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.03))
            q = t.transform.rotation
            return t.transform.translation.x, t.transform.translation.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except tf2_ros.TransformException: return None, None, None

    def get_rel_pose(self):
        """Helper to get our position relative to the enemy."""
        try:
            t = self.tf_buffer.lookup_transform(self.p['enemy_frame'], self.p['pursuer_frame'], rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.03))
            return t.transform.translation.x, t.transform.translation.y
        except: return None, None

    def compute_goal(self):
        """Computes the chasing goal. Yields control if in danger."""
        xe, ye = self.get_rel_pose()
        px, py, pyaw = self.get_pose(self.p['pursuer_frame'])
        
        if None in (xe, px): return None
        dist = math.hypot(xe, ye)

        # --- STATE MACHINE TRANSITIONS ---
        if self.state == 'CHASING' and xe > -0.2 and dist < self.p['evade_trigger_dist']:
            self.get_logger().warn("DANGER! Enemy head-on! Yielding to Evasion System.")
            self.state = 'EVADING'
            
            # Emergency brake & cancel tracking goal
            self.cmd_vel_pub.publish(Twist())
            if self.active_goal: self.active_goal.cancel_goal_async()
            self.last_sent_pose = None
            
        elif self.state == 'EVADING' and (dist > self.p['evade_clear_dist'] or xe < -0.4):
            self.get_logger().info("Danger clear. Resuming chase.")
            self.state = 'CHASING'
            if self.active_goal: self.active_goal.cancel_goal_async()
            self.last_sent_pose = None

        # --- GOAL LOGIC ---
        if self.state == 'EVADING':
            # FIX: We do absolutely nothing here now. 
            # We let the Evasion Calculator handle the math.
            return None
            
        else:
            # We are chasing. Get the Carrot TF and clamp it to the short goal distance.
            tx, ty, tyaw = self.get_pose(self.p['target_frame'])
            if tx is None: return None
            dx, dy, d = tx - px, ty - py, math.hypot(tx - px, ty - py)
            
            if d > self.p['short_goal_dist']:
                tx, ty = px + dx*(self.p['short_goal_dist']/d), py + dy*(self.p['short_goal_dist']/d)

        # Final check to ensure we don't send a goal inside a wall
        sx, sy = self.project_to_free_space(px, py, tx, ty)
        
        g = PoseStamped()
        g.header.frame_id, g.header.stamp = self.p['map_frame'], self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = sx, sy
        g.pose.orientation.z, g.pose.orientation.w = math.sin(tyaw/2.0), math.cos(tyaw/2.0)
        return g

    def control_loop(self):
        """Sends the goal to Nav2 if it has changed significantly."""
        if not self.nav_client.wait_for_server(timeout_sec=0.02): return
        
        goal = self.compute_goal()
        if not goal: return

        # Throttle Nav2 requests to prevent spamming the action server
        if not self.last_sent_pose or math.hypot(goal.pose.position.x - self.last_sent_pose.pose.position.x, goal.pose.position.y - self.last_sent_pose.pose.position.y) >= self.p['min_dist_threshold']:
            req = NavigateToPose.Goal()
            req.pose = goal
            fut = self.nav_client.send_goal_async(req)
            
            # Store the active goal handle so we can cancel it later if danger strikes
            fut.add_done_callback(lambda f: setattr(self, 'active_goal', f.result()) if f.result().accepted else None)
            self.last_sent_pose = goal

def main(args=None):
    rclpy.init(args=args)
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(ChaserController())
    try: exe.spin()
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
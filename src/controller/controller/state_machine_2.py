#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        self.state = 'CHASING'

        self.last_dist = None
        self.last_time = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.state_pub = self.create_publisher(String, 'our_bot/robot_state', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("State Machine Initialized (Velocity-Weighted Threat Assessment).")

    def get_pursuer_in_enemy_frame(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'enemy_bot/base_link', 'our_bot/base_link', 
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
            )
            return trans.transform.translation.x, trans.transform.translation.y
        except tf2_ros.TransformException:
            return None, None

    def evaluate_threat_state(self, rel_x: float, rel_y: float, now_sec: float):
        distance = math.hypot(rel_x, rel_y)
        relative_angle_deg = abs(math.degrees(math.atan2(rel_y, rel_x)))

        closing_speed = 0.0
        if self.last_dist is not None and self.last_time is not None:
            dt = now_sec - self.last_time
            if dt > 0.01:
                closing_speed = -(distance - self.last_dist) / dt
        
        self.last_dist = distance
        self.last_time = now_sec

        in_front_cone = relative_angle_deg <= 67.5       
        in_back_cone = relative_angle_deg >= 135.0       
        
        if in_front_cone:
            current_threshold = 2.8   
        elif in_back_cone:
            current_threshold = 0.0   
        else:
            current_threshold = 2.0   
            
        if distance > current_threshold:
            return False, distance, current_threshold

        # If enemy is close but actively opening distance rapidly, remain offensive
        if closing_speed < -0.2 and distance > 1.2:
            return False, distance, current_threshold
            
        return True, distance, current_threshold

    def control_loop(self):
        rel_x, rel_y = self.get_pursuer_in_enemy_frame()
        if rel_x is None: 
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        in_danger, dist, threshold = self.evaluate_threat_state(rel_x, rel_y, now_sec)

        if self.state == 'CHASING':
            if in_danger:
                self.get_logger().warn(f'DANGER ZONE! Dist: {dist:.2f}m <= {threshold}m. Switching to EVADING.')
                self.state = 'EVADING'

        elif self.state == 'EVADING':
            if not in_danger and dist > (threshold + 0.3):
                self.get_logger().info(f'Clear of threat (Dist: {dist:.2f}m). Resuming CHASING.')
                self.state = 'CHASING'

        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StateMachine())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
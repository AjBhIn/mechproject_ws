#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tf2_ros

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        self.state = 'CHASING'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state_pub = self.create_publisher(String, 'our_bot/robot_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/our_bot/cmd_vel', 10)

        # 10 Hz control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("State Machine Initialized with Custom Angular Threat Zones.")

    def emergency_brake(self):
        """Stops the robot dead before Nav2 calculates the evasion path."""
        self.cmd_vel_pub.publish(Twist())

    def get_pursuer_in_enemy_frame(self):
        """Gets our robot's position relative to the enemy robot."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'enemy_bot/base_link', 'our_bot/base_link', 
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.03)
            )
            return trans.transform.translation.x, trans.transform.translation.y
        except tf2_ros.TransformException:
            return None, None

    # ==========================================================
    # USER'S CUSTOM THREAT EVALUATION LOGIC
    # ==========================================================
    def evaluate_threat_state(self, rel_x: float, rel_y: float):
        """Dynamic Danger Zones to cover all 180-degree angles."""
        distance = math.hypot(rel_x, rel_y)
        relative_angle_deg = abs(math.degrees(math.atan2(rel_y, rel_x)))

        # 1. Define the angle zones
        in_front_cone = relative_angle_deg <= 67.5       # The main 135° front camera
        in_back_cone = relative_angle_deg >= 135.0       # The 90° rear zone
        
        # 2. Determine the danger radius based strictly on WHERE we are
        if in_front_cone:
            current_threshold = 2.8  # Very dangerous in front!
        elif in_back_cone:
            current_threshold = 0.0  # Safe in the back.
        else:
            current_threshold = 2.0  # Horizontal/Perpendicular flanks.
            
        # 3. Master Distance Check
        if distance > current_threshold:
            return False, distance, current_threshold
            
        return True, distance, current_threshold

    def control_loop(self):
        rel_x, rel_y = self.get_pursuer_in_enemy_frame()
        if rel_x is None: 
            return

        # Run your custom evaluation
        in_danger, dist, threshold = self.evaluate_threat_state(rel_x, rel_y)

        # --- STATE MACHINE TRANSITIONS ---
        if self.state == 'CHASING':
            # Trigger evasion immediately if we enter the custom danger thresholds
            if in_danger:
                self.get_logger().warn(f'DANGER ZONE! Dist: {dist:.2f}m <= {threshold}m. Switching to EVADING.')
                self.state = 'EVADING'
                self.emergency_brake()

        elif self.state == 'EVADING':
            # Clear evasion only when we are safely outside the threshold + a 0.3m hysteresis buffer
            if not in_danger and dist > (threshold + 0.3):
                self.get_logger().info(f'Clear of FOV (Dist: {dist:.2f}m). Resuming CHASING.')
                self.state = 'CHASING'

        # Broadcast state to Goal Sender and Evasion Calculator
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StateMachine())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
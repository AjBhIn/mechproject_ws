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

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.state_pub = self.create_publisher(String, 'our_bot/robot_state', 10)
        self.cmd_pub = self.create_publisher(Twist, 'our_bot/cmd_vel', 10)

        # Check threat every 0.1 seconds (10 Hz)
        self.create_timer(0.1, self.check_threat)

    def check_threat(self):
        try:
            # Look up enemy position relative to our robot
            transform = self.tf_buffer.lookup_transform(
                'enemy_bot/base_link', 
                'our_bot/base_link', 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.03)
            )
            rel_x = transform.transform.translation.x
            rel_y = transform.transform.translation.y
        except tf2_ros.TransformException:
            return

        distance = math.hypot(rel_x, rel_y)

        # Transition to EVADING if enemy is in front cone (rel_x > -0.2) and within 2.5m
        if self.state == 'CHASING' and rel_x > -0.2 and distance < 2.5:
            self.state = 'EVADING'
            self.get_logger().warn("Enemy detected head-on! Switching to EVADING")

            # Active steering turn away from enemy
            turn_cmd = Twist()
            turn_cmd.linear.x = 0.2
            turn_cmd.angular.z = 1.8 if rel_y < 0 else -1.8
            self.cmd_pub.publish(turn_cmd)

        # Transition back to CHASING if enemy is farther than 3.2m or drove past us (rel_x < -0.4)
        elif self.state == 'EVADING' and (distance > 3.2 or rel_x < -0.4):
            self.state = 'CHASING'
            self.get_logger().info("Danger cleared. Resuming CHASING")

        # Publish state string
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StateMachine())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
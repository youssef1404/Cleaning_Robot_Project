#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10.0)
        self.mode_publisher = self.create_publisher(String, 'robot_mode', 10.0)
        self.servo_publisher = self.create_publisher(String, 'servo_control', 10.0)
        
        self.twist = Twist()
        self.speed = 255.0
        self.turn_speed = 126.0
        self.mode = "manual" 

        # W - Move forward
        # A - Turn left
        # S - Move backward
        # D - Turn right
        # Q - Rotate servo left
        # E - Rotate servo right
        # R - Increase speed
        # F - Decrease speed
        # G - Switch to autonomous mode
        # H - Switch to manual (keyboard) mode
        # space - stop
        # CTRL+C to quit
        

    def set_mode(self, mode):
        """Set the robot mode (manual or autonomous)"""
        self.mode = mode
        msg = String()
        msg.data = mode
        self.mode_publisher.publish(msg)
        self.get_logger().info(f"Mode switched to {mode}")

    def handle_key(self, key):
        if self.mode == "manual":
            if key == 'w':
                self.twist.linear.x = self.speed
                self.twist.angular.z = 0.0
            elif key == 's':
                self.twist.linear.x = -self.speed
                self.twist.angular.z = 0.0
            elif key == 'a':
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.turn_speed
            elif key == 'd':
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.turn_speed
            elif key == ' ':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            elif key == 'r':
                self.speed = min(255, self.speed + 10.0)
                self.turn_speed = min(255, self.turn_speed + 10.0)
                self.get_logger().info(f"Speed increased: {self.speed}")
            elif key == 'f':
                self.speed = max(0.0, self.speed - 10.0)
                self.turn_speed = max(0.0, self.turn_speed - 10.0)
                self.get_logger().info(f"Speed decreased: {self.speed}")
            elif key == 'q':
                servo_msg = String()
                servo_msg.data = "left"
                self.servo_publisher.publish(servo_msg)
                self.get_logger().info("Servo moved left")
            elif key == 'e':
                servo_msg = String()
                servo_msg.data = "right"
                self.servo_publisher.publish(servo_msg)
                self.get_logger().info("Servo moved right")
            elif key == 'g':
                self.set_mode("autonomous")
            elif key == 'h':
                self.set_mode("manual")
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

            self.publisher.publish(self.twist)

    def main_loop(self):
        try:
            while True:
                key = input("Enter a key: ").lower()
                self.handle_key(key)
        except KeyboardInterrupt:
            self.get_logger().info("Exiting teleoperation.")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.main_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

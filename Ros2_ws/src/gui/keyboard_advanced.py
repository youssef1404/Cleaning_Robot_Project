#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import keyboard
import time

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10) #control wheels movement after receiving speeds
        self.mode_publisher = self.create_publisher(String, 'robot_mode', 10) #auto or manual
        self.servo_publisher = self.create_publisher(String, 'servo_control', 10) #control servo movement after receiving "left" / "right"
        
        self.twist = Twist()
        self.speed = 255.0
        self.turn_speed = 126.0
        self.mode = "manual" 
        self.running = True #####

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
        #Set the robot mode (manual or autonomous)
        self.mode = mode
        msg = String()
        msg.data = mode
        self.mode_publisher.publish(msg)
        self.get_logger().info(f"Mode switched to {mode}")

    def handle_keys(self):

        try:
            while self.running:
                # Reset twist values to stop motion unless a key is pressed
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                move_message = String()

                if keyboard.is_pressed('w'):  # Move forward
                    self.twist.linear.x = self.speed
                    self.twist.angular.z = 0.0
                    self.get_logger().info(f"Move Forward: {self.speed}")
                if keyboard.is_pressed('s'):  # Move backward
                    self.twist.linear.x = -self.speed
                    self.twist.angular.z = 0.0
                    self.get_logger().info(f"Move Backward: {self.speed}")
                if keyboard.is_pressed('a'):  # Turn left
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = self.turn_speed
                    self.get_logger().info(f"Turn Left: {self.speed}")
                if keyboard.is_pressed('d'):  # Turn right
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -self.turn_speed
                    self.get_logger().info(f"Turn Right: {self.speed}")

                # Speed control
                if keyboard.is_pressed('r'):  # Increase speed
                    self.speed = min(255, self.speed + 0.01)
                    self.turn_speed = min(255, self.turn_speed + 0.01)
                    self.get_logger().info(f"Speed increased: {self.speed}")
                if keyboard.is_pressed('f'):  # Decrease speed
                    self.speed = max(0.0, self.speed - 0.01)
                    self.turn_speed = max(0.0, self.turn_speed - 0.01)
                    self.get_logger().info(f"Speed decreased: {self.speed}")

                # Servo control
                if keyboard.is_pressed('q'):
                    servo_msg = String()
                    servo_msg.data = "left"
                    self.servo_publisher.publish(servo_msg)
                    self.get_logger().info("Servo moved left")
                if keyboard.is_pressed('e'):
                    servo_msg = String()
                    servo_msg.data = "right"
                    self.servo_publisher.publish(servo_msg)
                    self.get_logger().info("Servo moved right")

                # Mode switch
                if keyboard.is_pressed('g'):
                    self.set_mode("autonomous")
                if keyboard.is_pressed('h'):
                    self.set_mode("manual")

                # Stop all motion
                if keyboard.is_pressed('space'):
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.get_logger().info("STOP")

            self.publisher.publish(self.twist)
            time.sleep(0.1)
        
        except KeyboardInterrupt:
            self.get_logger().info("Exiting teleoperation.")
            self.running = False
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
    node.handle_keys()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

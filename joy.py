#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS3Controller:
    def __init__(self):
        self.cross_btn = 0
        self.circle_btn = 0
        self.triangle_btn = 0
        self.square_btn = 0
        self.L1_btn = 0
        self.R1_btn = 0
        self.L2_btn = 0
        self.R2_btn = 0
        self.joy_left_x = 0.0
        self.joy_left_y = 0.0
        self.joy_right_x = 0.0

    def get_data(self, msg):
        self.cross_btn = msg.buttons[0]
        self.circle_btn = msg.buttons[1]
        self.triangle_btn = msg.buttons[2]
        self.square_btn = msg.buttons[3]
        self.L1_btn = msg.buttons[4]
        self.R1_btn = msg.buttons[5]
        self.L2_btn = msg.buttons[6]
        self.R2_btn = msg.buttons[7]
        self.joy_left_x = msg.axes[0]
        self.joy_left_y = msg.axes[1]
        self.joy_right_x = msg.axes[3]

class MoveRobot:
    def __init__(self, joystick):
        self.joystick = joystick
        self.vx = 0.0
        self.vy = 0.0
        self.theta = 0.0
        self.speed_factor = 1.0
        self.slow_mode = 0.5
        self.fast_mode = 1.5
        self.normal_mode = 1.0

    def control(self):
        if self.joystick.R1_btn and self.joystick.R2_btn:
            self.speed_factor = self.slow_mode
        elif self.joystick.R1_btn:
            self.speed_factor = self.fast_mode
        else:
            self.speed_factor = self.normal_mode

        self.vx = self.joystick.joy_left_y * self.speed_factor
        self.vy = self.joystick.joy_left_x * self.speed_factor
        self.theta = self.joystick.joy_right_x * self.speed_factor

        return self.vx, self.vy, self.theta

class PS3ControllerNode(Node):
    def __init__(self):
        super().__init__('ps3_controller_node')
        self.joystick = PS3Controller()
        self.mover = MoveRobot(self.joystick)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("PS3 joystick controller node started.")

    def joy_callback(self, msg):
        self.joystick.get_data(msg)
        vx, vy, theta = self.mover.control()
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = theta
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    ps3_controller_node = PS3ControllerNode()

    try:
        rclpy.spin(ps3_controller_node)
    except KeyboardInterrupt:
        ps3_controller_node.get_logger().info('Node terminated by user.')
    finally:
        ps3_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


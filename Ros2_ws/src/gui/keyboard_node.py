import rclpy
from rclpy.node import Node 
from std_msgs.msg import Char
import keyboard

class Keyboard(Node):
    def __init__(self):
        super().__init__("keyboard_publisher")
        self.publisher = self.create_publisher(Char, 'cmd_drive', 10)
        self.get_logger().info("Keyboard Node Started")

    def send_command(self, command):
        msg = Char()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"command: {command}")
    

def main(args=None):
    rclpy.init(args=args)
    K_publisher = Keyboard()

    try:

        while rclpy.ok():
            if keyboard.is_pressed('w'):
                K_publisher.send_command('f')
            elif keyboard.is_pressed('s'):
                K_publisher.send_command('b')
            elif keyboard.is_pressed('a'): 
                K_publisher.send_command('l')
            elif keyboard.is_pressed('d'):  
                K_publisher.send_command('r')
            elif keyboard.is_pressed('space'):  # Stop / start
                K_publisher.send_command('s')
        
    except KeyboardInterrupt:
        pass

    K_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
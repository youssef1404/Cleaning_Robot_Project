import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import sys

try:
    import termios
    import tty
except ImportError:
    print("This script can only be run on Linux or Mac systems.")
    sys.exit(1)

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(Int32, 'key_input', 10)
        self.get_logger().info("Keyboard Publisher Node has started. Press keys to publish them.")

    def publish_key(self, key):
        msg = Int32()
        msg.data = key
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {key}')

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()

    try:
        while rclpy.ok():
            key = get_key()
            if key == '\x03':  # CTRL+C to exit
                break
            if key == 'w': # forward
                key = 1 
            if key == 's': # backward
                key = 2
            if key == 'd': # right
                key = 3
            if key == 'a': # left
                key = 4
            
            keyboard_publisher.publish_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_publisher.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
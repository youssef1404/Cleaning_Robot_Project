import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import sys

try:
    import termios
    import tty
except ImportError:
    print("This script can only be run on Linux or Mac systems.")
    sys.exit(1)

def get_key():
    """
    Capture a single keypress from the keyboard.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class KeyboardPublisher(Node):
    """
    ROS 2 Node to publish keyboard inputs as Int8 messages.
    """
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(Int8, '/key_input', 10)
        self.get_logger().info("Keyboard Publisher Node has started. Press keys to publish them.")

    def publish_key(self, key):
        """
        Publish the corresponding key value to the 'key_input' topic.
        """
        msg = Int8()
        msg.data = key
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {key}')

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()

    try:
        while rclpy.ok():
            key = get_key()

            # Map keys to specific integers
            key_mapping = {'w': 1, 's': 2, 'd': 3, 'a': 4, ' ': 5, 'b': 6, 'n':7}
            if key == '\x03':  # CTRL+C to exit
                break

            if key in key_mapping:
                keyboard_publisher.publish_key(key_mapping[key])
            else:
                keyboard_publisher.publish_key(0)
                keyboard_publisher.get_logger().warn(f"Unmapped key '{key}' pressed. Ignoring.")
    except KeyboardInterrupt:
        keyboard_publisher.get_logger().info("Shutting down Keyboard Publisher Node.")
    except Exception as e:
        keyboard_publisher.get_logger().error(f"An unexpected error occurred: {e}")
    finally:
        keyboard_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

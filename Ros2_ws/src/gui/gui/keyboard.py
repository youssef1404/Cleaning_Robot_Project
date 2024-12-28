import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8
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
        self.publisher_setPoint = self.create_publisher(Float32MultiArray, 'speed_setpoint', 10)
        self.publisher_keys = self.create_publisher(Int8, 'key_input', 10)
        self.get_logger().info("Keyboard Publisher Node has started. Press keys to publish them.")
        self.msgsent = [0.0,0.0]
        self.speed = 255.0

    def publish_setPoint(self, key):
        """
        Publish the corresponding key value to the 'key_input' topic.
        """
        msg = Float32MultiArray()
        msg.data = key
        self.publisher_setPoint.publish(msg)
        self.get_logger().info(f'Published: {key}')

    def publish_keys(self, key):
        """
        Publish the corresponding key value to the 'key_input' topic.
        """
        msg = Int8()
        msg.data = key
        self.publisher_keys.publish(msg)
        self.get_logger().info(f'Published: {key}')

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()

    try:
        while rclpy.ok():
            key = get_key()

            # Map keys to specific integers
            key_mapping_move = {'w': [keyboard_publisher.speed, keyboard_publisher.speed] ,
                            's': [-1 * keyboard_publisher.speed, -1 * keyboard_publisher.speed],
                            'd': [0.0,  keyboard_publisher.speed],
                            'a': [keyboard_publisher.speed, 0.0],
                            ' ': [0.0, 0.0]}

            # servo readings
            key_mappings = {'m': 1,
                            'n': 2,
                            'b': 3,
                            'v': 4}
            if key == '\x03':  # CTRL+C to exit
                break

            if key in key_mapping_move:
                keyboard_publisher.publish_setPoint(key_mapping_move[key])
            elif key in key_mappings:
                keyboard_publisher.publish_keys(key_mappings[key])
            else:
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


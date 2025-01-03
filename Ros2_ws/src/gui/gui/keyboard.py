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
        # self.publisher_setPoint = self.create_publisher(Float32MultiArray, 'speed_setpoint', 10)
        self.publisher_keys = self.create_publisher(Int8, '/key_input', 10)
        self.publisher_speeds = self.create_publisher(Int8, '/speed', 10)
        self.get_logger().info("Keyboard Publisher Node has started. Press keys to publish them.")
        self.msgsent = [0.0,0.0]
        self.speed = 255.0

    # def publish_setPoint(self, key):
    #     """
    #     Publish the corresponding key value to the 'key_input' topic.
    #     """
    #     msg = Float32MultiArray()
    #     msg.data = key
    #     self.publisher_setPoint.publish(msg)
    #     self.get_logger().info(f'Published: {key}')

    def publish_keys(self, key):
        """
        Publish the corresponding key value to the 'key_input' topic.
        """
        msg = Int8()
        msg.data = key
        self.publisher_keys.publish(msg)
        self.get_logger().info(f'Published: {key}')

    def publish_speeds(self, key):
        """
        Publish the corresponding key value to the 'key_input' topic.
        """
        msg = Int8()
        msg.data = key
        self.publisher_speeds.publish(msg)
        self.get_logger().info(f'Published: {key}')

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()

    try:
        while rclpy.ok():
            key = get_key()

            # Map keys to specific integers
            # key_mapping_move = {'w': [keyboard_publisher.speed, keyboard_publisher.speed] ,
            #                 's': [-1 * keyboard_publisher.speed, -1 * keyboard_publisher.speed],
            #                 'd': [0.0,  keyboard_publisher.speed],
            #                 'a': [keyboard_publisher.speed, 0.0],
            #                 ' ': [0.0, 0.0]}

            key_mappings = {'m': 1, # up servo
                            'n': 2, # down servo
                            'b': 3, # magnet on
                            'v': 4, # magnet off
                            'w': 5, # forward
                            's': 6, # backward
                            'd': 7, # right
                            'a': 8, # left
                            ' ': 9 # stop
                            } 
            
            speed_control = {'p' : 10, 'o' : 11}

            if key == '\x03':  # CTRL+C to exit
                break

            # if key in key_mapping_move:
            #     keyboard_publisher.publish_setPoint(key_mapping_move[key])
            if key in key_mappings:
                keyboard_publisher.publish_keys(key_mappings[key])
            elif key in speed_control:
                keyboard_publisher.publish_speeds(speed_control[key])
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


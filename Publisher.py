import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('detection_publisher')
        self.publisher_ = self.create_publisher(String, 'detection_state', 10)
        self.timer = self.create_timer(2.0, self.publish_detection_state)
        self.searching = True

    def publish_detection_state(self):
        msg = String()
        if self.searching:
            msg.data = "searching"
            self.get_logger().info("State: Searching for magnetic metal")
        else:
            msg.data = "found"
            self.get_logger().info("State: Magnetic metal found!")
        self.publisher_.publish(msg)
        self.searching = not self.searching  # Toggle state

def main(args=None):
    rclpy.init(args=args)
    node = DetectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

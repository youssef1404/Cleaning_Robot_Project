import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberSubscriber(Node):
    def __init__(self):
        # Initialize the Node with a name
        super().__init__('number_subscriber')

        # Create a subscription to the 'number_topic'
        self.subscription = self.create_subscription(
            Int32,  # Message type
            'number_topic',  # Topic name
            self.number_callback,  # Callback function
            10  # QoS (queue size)
        )
        self.subscription  # Prevent unused variable warning

    def number_callback(self, msg):
        # Callback function is executed whenever a message is received
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the NumberSubscriber node
    number_subscriber = NumberSubscriber()

    # Keep the node alive until interrupted
    rclpy.spin(number_subscriber)

    # Shutdown and clean up when exiting
    number_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

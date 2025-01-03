import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from playsound import playsound

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(
            String,
            'detection_state',
            self.play_audio_callback,
            10
        )
        self.subscription

    def play_audio_callback(self, msg):
        if msg.data == "searching":
            self.get_logger().info("Playing: Searching audio")
            playsound('searching.mp3')  # path
        elif msg.data == "found":
            self.get_logger().info("Playing: Found audio")
            playsound('found.mp3')  #  path

def main(args=None):
    rclpy.init(args=args)
    node = AudioSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

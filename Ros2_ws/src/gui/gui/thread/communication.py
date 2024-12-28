import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, Bool

class Comsystem(Node):
    def __init__(self, name="default",labels = None, sub_name="def", msgType=None):
        super().__init__(f"{name}_subscriber")

        self.subscriber_ = self.create_subscription(msgType, sub_name, self.updateData, 10)

        self.subscriber_
        self.labels = labels
        self.sub_name = sub_name

    def updateData(self, msg):
        
        # if self.sub_name == "/joo_topic": # low, medium , high
        #     self.labels.setText(str(msg.data))
        
        
        self.labels.setText(str(msg.data))

        self.get_logger().info(f"recieving {msg.data}")
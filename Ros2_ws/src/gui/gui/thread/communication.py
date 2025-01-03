import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, Bool, Float32MultiArray

class Comsystem(Node):
    def __init__(self, name="default",labels = None, sub_name="def", msgType=None):
        super().__init__(f"{name}_subscriber")

        self.subscriber_ = self.create_subscription(Float32MultiArray, sub_name, self.updateData, 10)

        self.subscriber_
        self.labels = labels
        self.sub_name = sub_name

    def updateData(self, msg):
        if type(self.labels == list):
            for i in range(len(self.labels)):
                self.labels[i].setText(str({msg.data[i]}))
        else:
            self.labels.setText(str(msg.data))

        self.get_logger().info(f"recieving {msg.data}")
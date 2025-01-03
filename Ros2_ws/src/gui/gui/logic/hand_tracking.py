import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8  
import cv2, sys
# import torch
# from ultralytics import YOLO
from rclpy.executors import MultiThreadedExecutor
from PySide6.QtWidgets import QWidget

sys.path.append("src/gui/gui/thread")
from camera import Camera

sys.path.append("src/gui/gui/ui")
from detection_ui import Ui_Form as View

class HandTracking(QWidget, View, Node):
    def __init__(self, parent = None, executor:MultiThreadedExecutor = None):
        QWidget.__init__(self, parent)      # Initialize Qt widget functionality
        View.__init__(self)                 # Initialize the UI view
        Node.__init__(self, 'Hand_Trancking_Node')   # Initialize ROS2 node functionality

        self.setupUi(self)
        self.executor = executor

        # Publishers
        self.publisher = self.create_publisher(Int8, '/key_input', 10)

        self.cam = Camera(name='main', address = 0, label=self.camera_label)

        
        self.executor.add_node(self.cam)
        self.executor.add_node(self)



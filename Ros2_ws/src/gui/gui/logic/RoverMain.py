#!/usr/bin/env python3

from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer, QTime
from PySide6 import QtGui
import sys, os
from std_msgs.msg import Int8, Float32, Bool, Int32, Float32MultiArray
from rclpy.node import Node

# from ui.rover_main import Ui_Form as View
# from thread.camera import Camera
# from thread.timer import Timer
# from thread.communication import Comsystem

sys.path.append("src/gui/gui/ui")
from rover_main import Ui_Form as View
sys.path.append("src/gui/gui/thread")
from camera import Camera
from timer import Timer
from communication import Comsystem

from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtGui import QImage, QPixmap
import cv2 as cv

address = "http://192.168.5.36:8080/video"

class Rover_Main(QWidget, View, Node):
    def __init__(self, parent = None, executor:MultiThreadedExecutor = None):
        QWidget.__init__(self, parent)      # Initialize Qt widget functionality
        View.__init__(self)                 # Initialize the UI view
        Node.__init__(self, 'rover_main')   # Initialize ROS2 node functionality

        self.setupUi(self)
        self.excutor = executor
        
        # Initialize nodes and components as before
        self.webcam = Camera(name='main', address = address, label=self.camera_label)
        self.timer = Timer(label=self.timeLabel)
        self.speed_feedback = Comsystem(name="speed_feedback", labels=[self.speed_label_1, self.speed_label_2], sub_name="speed_feedback")
        self.encoder_counts = Comsystem(name="encoder_counts", labels=[self.count_label_1, self.count_label_2], sub_name="counts")
        
        # Add new publisher for mode state
        self.mode_publisher = self.create_publisher(Int8, 'mode_state', 10)
        
        # Add nodes to executor
        self.excutor.add_node(self.webcam)
        self.excutor.add_node(self.timer)
        self.excutor.add_node(self.speed_feedback)
        self.excutor.add_node(self.encoder_counts)
        self.excutor.add_node(self)
        
        # Connect buttons
        self.screenButton.clicked.connect(self.captureScreenshot)
        # self.stopButton.clicked.connect(self.stop_feed)
        self.auto_button.clicked.connect(self.enableAuto)
        self.manuel_button.clicked.connect(self.enableManuel)
        
        self.frame = None
        self.state = 0  # default manuel
        self.frameCount = 0
        self.rootPath = os.path.join(os.getcwd(), 'src', 'gui', 'gui', 'photos')
        os.makedirs(self.rootPath, exist_ok=True)

    def captureScreenshot(self):
        if self.webcam.getCurrentFrame() is not None:
            self.frameCount += 1
            self.filename = 'Screenshot_'+ str(self.frameCount) +'.png'  # Changed space to underscore
            image_path = os.path.join(self.rootPath, self.filename)
            
            self.frame = self.webcam.getCurrentFrame()
            self.frame = cv.rotate(self.frame, cv.ROTATE_90_COUNTERCLOCKWISE)
            cv.imwrite(image_path, self.frame)
            self.get_logger().info(f"Successfully captured screenshot at {image_path}")
        else:
            self.get_logger().warn("Can't captured screenshot !!!")

    def stop_feed(self):
        self.webcam.stop()

    def enableAuto(self):
        """Enable autonomous mode and publish state"""
        self.state = 1
        msg = Int8()
        msg.data = self.state
        self.mode_publisher.publish(msg)
        self.get_logger().info(f"Enable Autonomous mode, published {self.state}")

    def enableManuel(self):
        """Enable manual mode and publish state"""
        self.state = 0
        msg = Int8()
        msg.data = self.state
        self.mode_publisher.publish(msg)
        self.get_logger().info(f"Enable manual mode, published {self.state}")

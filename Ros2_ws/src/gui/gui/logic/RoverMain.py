#!/usr/bin/env python3

from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer, QTime
from PySide6 import QtGui
import sys, os
from std_msgs.msg import Int16, Float32, Bool, Int32
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



address = "http://192.168.238.223:8080/video"

class Rover_Main(QWidget, View):
    def __init__(self, parent = None, executor:MultiThreadedExecutor = None):
        super(Rover_Main, self).__init__(parent)
        self.setupUi(self)
        self.excutor = executor

        # self.publisher_ = self.create_publisher(Int16, '/motion_state', 10)

        self.webcam = Camera(name='main', address = 0, label=self.camera_label)
        self.timer = Timer(label=self.timeLabel)
        # self.tt = Comsystem(name="fuck", labels=self.speed_label_2, sub_name='/joo_topic', msgType=Int32)

        self.excutor.add_node(self.webcam)
        self.excutor.add_node(self.timer)
        # self.excutor.add_node(self.tt)

        self.screenButton.clicked.connect(self.captureScreenshot)
        # self.auto_button.clicked.connect(self.enableAuto)
        # self.manuel_button.clicked.connect(self.enableManuel)

        self.frame = None
        self.state = 0 # default manuel
        self.frameCount = 0
        self.rootPath = os.path.join(os.getcwd(), '..', 'photos')

    def captureScreenshot(self): 
        if self.webcam.getCurrentFrame() is not None:
            self.frameCount += 1
            self.filename = 'Screenshot '+ str(self.frameCount) +'.png'
            image_path = os.path.join(self.rootPath, self.filename)
            self.frame = self.webcam.getCurrentFrame()
            cv.imwrite(image_path ,self.frame)

            image = cv.cvtColor(self.frame, cv.COLOR_BGR2RGB)
            FilppedImage = cv.flip(image, 1)
            convertToQTformat = QImage(FilppedImage.data, FilppedImage.shape[1], FilppedImage.shape[0], QImage.Format_RGB888)
            self.webcam.setPixmap(QPixmap.fromImage(convertToQTformat))

    # def enableAuto(self):
    #     self.state = 1
    #     self.publisher_.publish(self.state)
    #     self.get_logger().info(f"Enable Autonomous mode, publish {self.state}")

    # def enableManuel(self):
    #     self.state = 0
    #     self.publisher_.publish(self.state)
    #     self.get_logger().info(f"Enable manuel mode, publish {self.state}")

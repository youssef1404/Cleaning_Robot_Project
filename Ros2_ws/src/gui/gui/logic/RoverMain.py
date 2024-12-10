#!/usr/bin/env python3

from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer, QTime
import sys
from std_msgs.msg import Int16, Float32, Bool, Int32

from ui.rover_main import Ui_Form as View
from thread.camera import Camera
from thread.timer import Timer
from thread.communication import Comsystem

from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtGui import QImage, QPixmap

address = "http://192.168.0.102:8080/video"
class Rover_Main(QWidget, View):
    def __init__(self, parent = None, executor:MultiThreadedExecutor = None):
        super(Rover_Main, self).__init__(parent)
        self.setupUi(self)
        self.excutor = executor

        self.webcam = Camera(name='main', address = 0, label=self.camera_label)
        self.timer = Timer(label=self.timeLabel)
        self.tt = Comsystem(name="fuck", labels=self.speed_label_2, sub_name='/joo_topic', msgType=Int32)

        self.excutor.add_node(self.webcam)
        self.excutor.add_node(self.timer)
        self.excutor.add_node(self.tt)


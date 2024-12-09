#!/usr/bin/env python3

from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer, QTime
import sys
from std_msgs.msg import Int16, Float32, Bool

from ui.rover_main import Ui_Form as View
from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtGui import QImage, QPixmap


class Rover_Main(QWidget, View):
    def __init__(self, parent=None, executor:MultiThreadedExecutor = None, camera=None):
        super(Rover_Main, self).__init__(parent)
        self.setupUi(self)
        self.excutor = executor


#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2 as cv
from PySide6.QtCore import *
from PySide6.QtGui import *
from utilss.utils import Utils

class Camera(Node):
    def __init__(self, name, address, label):
        super().__init__(f"{name.lower()}_camera_frames")
        self.name = name.lower()
        self.address = address
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.cap = cv.VideoCapture(self.address)
        self.label = label

    def timer_callback(self):    
        ret, frame = self.cap.read()
        if ret:
            try: 
                self.label.setPixmap(QPixmap.fromImage(Utils.arrayToQImage(frame)))
            except:
                print(f"{self.name}: No connection trying to reconnect....")
                print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")

        self.get_logger().info(f'Reading {self.name} camera frames')     

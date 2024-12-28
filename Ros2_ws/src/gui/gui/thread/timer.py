#!/usr/bin/env python3

import sys
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer, QTime
import sys
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer, QTime

from rclpy.node import Node
from datetime import datetime, timedelta

class Timer(Node):
    def __init__(self, parent=None, period=15, label="default"):
        super().__init__("timer")
        self.label = label

        self.end_time = datetime.now() + timedelta(minutes=period)
        
        self.timer = QTimer(parent)
        self.timer.timeout.connect(self.updateTimer)
        self.timer.start(1000)  # Update every second

    def updateTimer(self):
        remaining_time = self.end_time - datetime.now()
        # print(remaining_time)
        if remaining_time.total_seconds() > 0:
            time_str = str(remaining_time).split('.')[0][2:]  # Format timedelta as MM:SS
            self.label.setText(time_str)
        else:
            self.label.setText("Time's up!")
            self.timer.stop()  # Stop the QTimer when time is up

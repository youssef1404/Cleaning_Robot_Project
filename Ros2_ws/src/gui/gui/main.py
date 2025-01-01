#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import sys
from rclpy.executors import MultiThreadedExecutor
from PySide6.QtWidgets import QMainWindow, QApplication
from PySide6.QtCore import *
from PySide6.QtGui import *

sys.path.append("src/gui/gui/ui")
from main_window import Ui_MainWindow as View 

sys.path.append("src/gui/gui/logic")
from RoverMain import Rover_Main 
from detection import ObjectTracking

sys.path.append("src/gui/gui/utils")
from utilss import Utils

from threading import Thread

class MyGUI(QMainWindow, View):
    def __init__(self, parent=None):
        super(MyGUI, self).__init__(parent)
        self.setupUi(self)

        self.setWindowTitle("Bor3y Main")

        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.RoverMainTab = Rover_Main(executor = self.executor)
        self.DetectionTab = ObjectTracking(executor = self.executor)

        self.tabWidget.addTab(self.RoverMainTab, "Main")
        self.tabWidget.addTab(self.DetectionTab, "Detection")

        executor_thread = Thread(target=self.executor.spin, daemon=True)
        executor_thread.start()

def main(args=None):
    app = QApplication(sys.argv)
    window = MyGUI()
    window.show()
    app.exec()

if __name__ == "__main__":
    main()

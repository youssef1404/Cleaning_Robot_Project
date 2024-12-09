#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import sys
from rclpy.executors import MultiThreadedExecutor
from PySide6.QtWidgets import QMainWindow, QApplication

from ui.main_window import Ui_MainWindow as View

# sys.path.append("/home/youssef/gui_ws/src/gui/gui/logic")
from logic.main_tab import Rover_Main 
from threading import Thread

class MyGUI(QMainWindow, View):
    def __init__(self, parent=None):
        super(MyGUI, self).__init__(parent)
        self.setupUi(self)

        self.setWindowTitle("Bor3y Main")

        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.RoverMainTab = Rover_Main(executor=self.executor)

        self.tabWidget.addTab(self.RoverMainTab, "Main")

        executor_thread = Thread(target=self.executor.spin, daemon=True)
        executor_thread.start()

def main(args=None):
    app = QApplication(sys.argv)
    window = MyGUI()
    window.show()
    app.exec()

if __name__ == "__main__":
    main()
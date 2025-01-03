import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8  
import cv2, sys
# import torch
# from ultralytics import YOLO
from rclpy.executors import MultiThreadedExecutor
from PySide6.QtWidgets import QWidget
import cv2
from PySide6.QtCore import *
from PySide6.QtGui import *
import sys
import mediapipe as mp

sys.path.append("src/gui/gui/thread")
from camera import Camera

sys.path.append("src/gui/gui/ui")
from detection_ui import Ui_Form as View


sys.path.append("src/gui/gui/utils")
from utilss import Utils

# Mediapipe setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2)  # Ensure we process both hands
mp_draw = mp.solutions.drawing_utils

key_mappings = { 1 : 'Up servo', # up servo
                2 : 'Down Servo', # down servo
                3 : 'Magnet ON', # magnet on
                4 : 'Magnet OFF', # magnet off
                5 : 'Forward', # forward
                6 : 'Backward', # backward
                7 : 'Right', # right
                8 : 'Left', # left
                9 : 'stop' # stop
                } 

class HandTracking(QWidget, View, Node):
    def __init__(self, parent = None, executor:MultiThreadedExecutor = None):
        QWidget.__init__(self, parent)      # Initialize Qt widget functionality
        View.__init__(self)                 # Initialize the UI view
        Node.__init__(self, 'Hand_Trancking_Node')   # Initialize ROS2 node functionality

        self.setupUi(self)
        self.executor = executor

        # Publishers
        self.publisher = self.create_publisher(Int8, '/key_input', 10)
        self.cap = cv2.VideoCapture(0)  # Open the default camera
        self.timer = self.create_timer(0.1, self.process_frame)  # Timer callback to process frames at 10 Hz
        self.count = 0  # Initialize finger count
        self.metals = 0
        self.flag = 0

        self.executor.add_node(self.cap)
        self.executor.add_node(self)

        self.box_label.setText(str(self.metals))


    def process_frame(self):
        success, img = self.cap.read()
        if not success:
            self.get_logger().error('Failed to capture image')
            return
        
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(imgRGB)

        # Initialize finger count
        total_fingers = 0

        # Check if hands are detected
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                fingers = []

                # Check for each finger (index, middle, ring, pinky) if it's up
                fingers.append(self.get_finger_up(handLms, mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.INDEX_FINGER_DIP))
                fingers.append(self.get_finger_up(handLms, mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_DIP))
                fingers.append(self.get_finger_up(handLms, mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.RING_FINGER_DIP))
                fingers.append(self.get_finger_up(handLms, mp_hands.HandLandmark.PINKY_TIP, mp_hands.HandLandmark.PINKY_DIP))

                # Special case for thumb (requires checking the X coordinate)
                thumb_tip = handLms.landmark[mp_hands.HandLandmark.THUMB_TIP]
                thumb_ip = handLms.landmark[mp_hands.HandLandmark.THUMB_IP]
                if thumb_tip.x > thumb_ip.x:  # Thumb is extended outwards
                    fingers.append(1)
                else:
                    fingers.append(0)

                # Count the number of fingers that are up for this hand
                total_fingers += fingers.count(True)

                # Draw landmarks and connections on the image
                mp_draw.draw_landmarks(img, handLms, mp_hands.HAND_CONNECTIONS)

        # Publish the total number of fingers up (0 to 10)
        self.count = total_fingers
        self.get_logger().info(f'Fingers up: {self.count}')

        msg = Int8()
        msg.data = self.count
        self.publisher_.publish(msg)

        self.command_label.setText(key_mappings[self.count])

        if self.count == 1 and not self.flag:
            self.metals += 1
            self.box_label.setText(str(self.metals))
            self.flag = 1

        if self.count == 2:
            self.flag = 0

        # Display the count on the image
        cv2.putText(img, f'number: {self.count}', (50, 50), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 0, 0), 2)

        self.camera_label.setPixmap(QPixmap.fromImage(Utils.arrayToQImage(img)))
        # # Display the image
        # cv2.imshow("Image", img)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.cap.release()
        #     cv2.destroyAllWindows()
        #     rclpy.shutdown()

    def get_finger_up(self, hand_landmarks, finger_tip, finger_bottom):

        if hand_landmarks.landmark[finger_tip].y < hand_landmarks.landmark[finger_bottom].y:
            return True
        else:
            return False
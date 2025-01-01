import rclpy
from rclpy.node import Node
from std_msgs.msg import String  
import cv2, sys
# import torch
# from ultralytics import YOLO
from rclpy.executors import MultiThreadedExecutor
from PySide6.QtWidgets import QWidget


sys.path.append("src/gui/gui/ui")
from detection_ui import Ui_Form as View

class ObjectTracking(QWidget, View, Node):
    def __init__(self, parent = None, executor:MultiThreadedExecutor = None):
        QWidget.__init__(self, parent)      # Initialize Qt widget functionality
        View.__init__(self)                 # Initialize the UI view
        Node.__init__(self, 'Object_tracking_node')   # Initialize ROS2 node functionality

        # Parameters
        self.movement_topic = '/movement'

        # Publishers
        self.publisher = self.create_publisher(String, self.movement_topic, 10)

        # Check if CUDA (GPU) is available
        # self.device = "cuda" if torch.cuda.is_available() else "cpu"
        # self.get_logger().info(f"Using device: {self.device}")

        # Load YOLO model
        # self.model = YOLO("best.pt") 

        # URL of IP Webcam video stream
        self.url = 'http://192.168.1.104:8080/video' 

        # Open the video stream
        # self.cap = cv2.VideoCapture(self.url)

        # Check if the video stream was opened successfully
        # if not self.cap.isOpened():
        #     self.get_logger().error("Error: Unable to connect to the video stream.")
        #     exit()

        # Movement Variables
        self.screen_center = (0, 0)
        self.object_center = None

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to grab frame.')
            return

        height, width, _ = frame.shape
        self.screen_center = (width // 2, height // 2)

        # Make predictions with YOLO on the current frame
        # results = self.model(frame)  

        # Annotate the frame with YOLO results (bounding boxes, labels)
        # annotated_frame = results[0].plot()

        # # Analyze results
        # for box in results[0].boxes:
        #     x1, y1, x2, y2 = map(int, box.xyxy[0])
        #     self.object_center = ((x1 + x2) // 2, (y1 + y2) // 2)
        #     break  # Exit after processing the first detected object
        # else:
        #     self.object_center = None

        # Control Movement
        self.move_robot()

        # Display the annotated frame
        # cv2.imshow('Annotated Frame', annotated_frame)

        # Wait for 50 milliseconds 
        if cv2.waitKey(50) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def move_robot(self):
        command = ""

        if self.object_center is not None:
            # Calculate error
            error_x = self.object_center[0] - self.screen_center[0]

            # Control logic
            if abs(error_x) < 20:  # Tolerance for centered object
                command = "forward"
            else:
                command = "right" if error_x > 0 else "left"
        else:
            command = "explore"

        # Publish movement commands
        self.publisher.publish(String(data=command))  # Publish the command as a String message
        self.get_logger().info(f"{command}")

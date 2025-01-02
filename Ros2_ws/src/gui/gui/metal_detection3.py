import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import cv2
import torch
from ultralytics import YOLO

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')

        # Parameters
        self.movement_topic = '/movement'
        self.frame_interval = 10  # Process every 10th frame
        self.frame_count = 0
        self.previous_object_detected = False
        self.instructions_executed = False

        # Publishers
        self.publisher = self.create_publisher(Int8, self.movement_topic, 10)

        # Check if CUDA (GPU) is available
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # Load YOLO model
        self.model = YOLO("best.pt") 

        # URL of IP Webcam video stream
        self.url = 'http://192.168.1.146:8080/video' 

        # Open the video stream
        self.cap = cv2.VideoCapture(self.url)

        # Check if the video stream was opened successfully
        if not self.cap.isOpened():
            self.get_logger().error("Error: Unable to connect to the video stream.")
            exit()

        # Movement Variables
        self.screen_center = (0, 0)
        self.object_center = None

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to grab frame.')
            return

        # Process every 'frame_interval' frame
        self.frame_count += 1
        if self.frame_count % self.frame_interval != 0:
            return

        height, width, _ = frame.shape
        self.screen_center = (width // 2, height // 2)

        # Make predictions with YOLO on the current frame
        results = self.model(frame)  

        # Annotate the frame with YOLO results (bounding boxes, labels)
        annotated_frame = results[0].plot()

        # Analyze results
        object_detected = False
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            self.object_center = ((x1 + x2) // 2, (y1 + y2) // 2)
            object_detected = True
            break  # Exit after processing the first detected object
        else:
            self.object_center = None

        # Compare with previous detection state
        if object_detected != self.previous_object_detected:
            self.get_logger().info("Object detection state changed.")
            self.previous_object_detected = object_detected
            self.instructions_executed = False

        # Control Movement
        self.move_robot(object_detected)

        # Display the annotated frame
        cv2.imshow('Annotated Frame', annotated_frame)

        # Wait for 50 milliseconds 
        if cv2.waitKey(10) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def move_robot(self, object_detected):
        command = ""

        if object_detected:
            if not self.instructions_executed:
                # Execute instructions when object is detected
                commands = [
                    9,
                    2,
                    3,
                    1,
                    4
                ]
                for cmd in commands:
                    self.publisher.publish(Int8(data=cmd))
                    self.get_logger().info(f"Executing: {cmd}")
                    rclpy.spin_once(self, timeout_sec=1.0)  # Small delay for execution
                self.instructions_executed = True
            command = 9
        else:
            command = 7

        # Publish movement commands
        self.publisher.publish(Int8(data=command))  # Publish the command as a String message
        self.get_logger().info(f"{command}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    try:
        while rclpy.ok():
            node.process_frame()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

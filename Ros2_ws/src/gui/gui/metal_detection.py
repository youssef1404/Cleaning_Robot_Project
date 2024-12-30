import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import String message type
import cv2
import torch
from ultralytics import YOLO

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')

        # Parameters
        self.cmd_vel_topic = '/cmd_vel'

        # Publishers
        self.publisher = self.create_publisher(String, self.cmd_vel_topic, 10)

        # Check if CUDA (GPU) is available
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # Load YOLO model
        self.model = YOLO("best.pt")  # Load your model file

        # URL of your IP Webcam video stream
        self.url = 'http://192.168.1.104:8080/video'  # Replace with your IP Webcam URL

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

        height, width, _ = frame.shape
        self.screen_center = (width // 2, height // 2)

        # Make predictions with YOLO on the current frame
        results = self.model(frame)  # Inference on the frame

        # Annotate the frame with YOLO results (bounding boxes, labels)
        annotated_frame = results[0].plot()

        # Analyze results
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            self.object_center = ((x1 + x2) // 2, (y1 + y2) // 2)
            break  # Exit after processing the first detected object
        else:
            self.object_center = None

        # Control Movement
        self.move_robot()

        # Display the annotated frame
        cv2.imshow('Annotated Frame', annotated_frame)

        # Wait for 50 milliseconds and check if 'q' is pressed to exit
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
                # self.get_logger().info("forward")
            else:
                command = "right" if error_x > 0 else "left"
        else:
            command = "explore"

        # Publish movement commands
        self.publisher.publish(String(data=command))  # Publish the command as a String message
        self.get_logger().info(f"move {command}")

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
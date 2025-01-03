import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import cv2
import mediapipe as mp

# Mediapipe setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2)  # Ensure we process both hands
mp_draw = mp.solutions.drawing_utils

# ROS2 Publisher setup
class HandGesturePublisher(Node):

    def __init__(self):
        super().__init__('hand_gesture_publisher')  # ROS2 Node name
        self.publisher_ = self.create_publisher(Int8, 'key_input', 10)  # Topic to publish finger count
        self.cap = cv2.VideoCapture(0)  # Open the default camera
        self.timer = self.create_timer(0.1, self.process_frame)  # Timer callback to process frames at 10 Hz
        self.count = 0  # Initialize finger count

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

        # Display the count on the image
        cv2.putText(img, f'number: {self.count}', (50, 50), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 0, 0), 2)

        # Display the image
        cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def get_finger_up(self, hand_landmarks, finger_tip, finger_bottom):

        if hand_landmarks.landmark[finger_tip].y < hand_landmarks.landmark[finger_bottom].y:
            return True
        else:
            return False

def main(args=None):
    rclpy.init(args=args)
    node = HandGesturePublisher()
    rclpy.spin(node)  # Keep the node running
    node.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

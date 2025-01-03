import cv2
import mediapipe as mp
import time

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
pTime = 0
count = 0 
right_hand_count = 0
left_hand_count = 0 

def get_finger_up(hand_landmarks, finger_tip, finger_bottom):
    """
    Checks if a finger is up by comparing the y-coordinates of the tip and bottom landmarks.

    Args:
        hand_landmarks: Hand landmarks detected by MediaPipe.
        finger_tip: Index of the finger tip landmark.
        finger_bottom: Index of the finger bottom landmark.

    Returns:
        True if the finger is up, False otherwise.
    """
    if hand_landmarks.landmark[finger_tip].y < hand_landmarks.landmark[finger_bottom].y:
        return True
    else:
        return False

while True:
    success, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    right_hand_count = 0
    left_hand_count = 0 

    if results.multi_hand_landmarks:
        for hand_idx, handLms in enumerate(results.multi_hand_landmarks): 
            fingers = []
            # Check if index, middle, ring, and pinky fingers are up
            fingers.append(get_finger_up(handLms, mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.INDEX_FINGER_DIP))
            fingers.append(get_finger_up(handLms, mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_DIP))
            fingers.append(get_finger_up(handLms, mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.RING_FINGER_DIP))
            fingers.append(get_finger_up(handLms, mp_hands.HandLandmark.PINKY_TIP, mp_hands.HandLandmark.PINKY_DIP))

            # Check if thumb is up
            thumb_tip = handLms.landmark[mp_hands.HandLandmark.THUMB_TIP]
            thumb_ip = handLms.landmark[mp_hands.HandLandmark.THUMB_IP]
            if thumb_tip.x > thumb_ip.x:  # Thumb is extended outwards
                fingers.append(1)
            else:
                fingers.append(0)

            # Count the number of fingers up for each hand
            if hand_idx == 0:  # Right hand
                right_hand_count = fingers.count(True)
            elif hand_idx == 1:  # Left hand
                left_hand_count = fingers.count(True) 

            # Draw landmarks 
            mp_draw.draw_landmarks(img, handLms, mp_hands.HAND_CONNECTIONS)

        # Calculate the combined count
        count = right_hand_count + left_hand_count 

    # Display the count 
    cv2.putText(img, "Herzlich wilkommen", (100, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (0, 0,250), 1) 
    cv2.putText(img, str(count), (50, 50), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 0, 0), 3) 

    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
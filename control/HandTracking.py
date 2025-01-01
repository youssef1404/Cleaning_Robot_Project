import cv2
import mediapipe as mp
import time

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
pTime = 0
count = 0 

def get_finger_up(hand_landmarks, finger_tip, finger_bottom):
    if hand_landmarks.landmark[finger_tip].y < hand_landmarks.landmark[finger_bottom].y:
        return True
    else:
        return False


while True:
    success, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)
    cv2.putText(img, "Hallo und Guten Abend", (160, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3) 
    cv2.putText(img, "in der Bor3y-Welt :\" ", (160, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3) 

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            fingers = []
            # Check if index, middle, ring, and pinky fingers are up
            fingers.append(get_finger_up(handLms, mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.INDEX_FINGER_DIP))
            fingers.append(get_finger_up(handLms, mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_DIP))
            fingers.append(get_finger_up(handLms, mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.RING_FINGER_DIP))
            fingers.append(get_finger_up(handLms, mp_hands.HandLandmark.PINKY_TIP, mp_hands.HandLandmark.PINKY_DIP))

            thumb_tip = handLms.landmark[mp_hands.HandLandmark.THUMB_TIP]
            thumb_ip = handLms.landmark[mp_hands.HandLandmark.THUMB_IP]
            if thumb_tip.x > thumb_ip.x:  # Thumb is extended outwards
                fingers.append(1)
            else:
                fingers.append(0)

            # Count the number of fingers up
            count = fingers.count(True)
            print('the count is : ',count)
            # Draw landmarks and display the count 
            mp_draw.draw_landmarks(img, handLms, mp_hands.HAND_CONNECTIONS)
            cv2.putText(img, str(count), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3) 

    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
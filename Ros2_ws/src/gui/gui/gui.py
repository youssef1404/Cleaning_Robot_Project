import cv2

# Replace with the IP address shown in the IP Webcam app
ip_address = "http://192.168.0.107:8080/video"

# Capture video from the IP Webcam
cap = cv2.VideoCapture(ip_address)

while True:
    # Read the video stream
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Display the video
    cv2.imshow("IP Webcam", frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()

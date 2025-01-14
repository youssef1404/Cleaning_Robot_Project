
# Metal Collector Robot

## Overview
The Metal Collector Robot is an innovative project designed to autonomously or manually detect and collect metal objects and place them into a designated box. This system combines robotics, computer vision, and ROS2 for seamless operation and interaction between hardware and software components.

---

## Features
1. **Metal Detection and Collection**
   - Detects metal objects using a webcam and YOLO model.
   - Collects detected metals using a mechanical system and deposits them into a box.

2. **Operational Modes**
   - **Manual Mode:** Control the robot using a keyboard interface on the laptop.
   - **Autonomous Mode:** Automatically detects and collects metals using the YOLO model and a webcam.

3. **Communication Architecture**
   - **Laptop:**
     - Acts as the central control hub, running the ROS2 server.
     - Hosts the Graphical User Interface (GUI) that allows users to visualize camera feedback, monitor sensor data, and control the robot.
     - The ROS2 server facilitates communication between various nodes, such as those for YOLO-based detection, manual control, and sensor feedback.
   
   - **ESP32:**
     - Functions as the robot's onboard controller, handling real-time tasks such as motor control and sensor data acquisition.
     - Equipped with Micro-ROS, which ensures efficient and reliable communication with the ROS2 server on the laptop.
     - Receives commands (e.g., movement or mode changes) from the ROS2 server and sends sensor data back to the laptop for monitoring and decision-making.

4. **Graphical User Interface (GUI)**
   - Displays live camera feed.
   - Shows sensor data in real-time.
   - Allows mode selection (manual or autonomous).
   - Provides control options for robot operation.

---

## System Architecture
1. **Hardware Components**
   - **ESP32:** Acts as the robot's microcontroller for movement and sensor processing.
   - **Webcam:** Captures video feed for YOLO-based object detection.
   - **Motors and Mechanism:** Enables movement and collection of metal objects.

2. **Software Components**
   - **ROS2:** Manages communication between nodes on the laptop.
   - **Micro-ROS:** Facilitates communication between ESP32 and the ROS2 server.
   - **YOLO Model:** Processes webcam feed to detect metal objects.
   - **Keyboard Node:** Allows manual robot control via the laptop.
   - **GUI Application:** Provides a user-friendly interface for operation and monitoring.

---

## Installation and Setup
### Prerequisites
- ROS2 installed on the laptop.
- Micro-ROS library configured for ESP32.
- YOLO model dependencies installed.

### Steps
1. **ESP32 Configuration**
   - Flash Micro-ROS firmware onto ESP32.
   - Connect sensors and motors as per the wiring diagram.

2. **ROS2 Workspace**
   - Clone the repository and build the ROS2 workspace:
     ```bash
     mkdir -p ~/metal_collector_ws/src
     cd ~/metal_collector_ws/src
     git clone <repository-url>
     cd ..
     colcon build
     ```
   - Source the workspace:
     ```bash
     source ~/metal_collector_ws/install/setup.bash
     ```

3. **Launching Nodes**
   - Running the Agent:
     ```bash
     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 
     ```
   - Launch the GUI:
     ```bash
     ros2 run gui guiNode
     ```
   - Run the YOLO detection node:
     ```bash
     ros2 run gui yolo_node
     ```

---

## Usage
1. **Manual Mode**
   - Use the keyboard node to control the robot manually:
     ```bash
     ros2 run gui keyboard_publisher
     ```

2. **Autonomous Mode**
   - Activate autonomous mode from the GUI or by publishing a command:
     ```bash
     ros2 topic pub /robot_mode std_msgs/String "data: 'autonomous'"
     ```

3. **Monitoring and Control**
   - Use the GUI to:
     - View live camera feed.
     - Monitor sensor data.
     - Switch between manual and autonomous modes.

---

## Future Enhancements
- Integrating additional sensors for enhanced navigation.
- Adding a feature to sort metals by type.
- Improving YOLO model accuracy for specific metal detection.
- Implementing a mobile app for remote control and monitoring.

---

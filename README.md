# Cleaning_Robot_Project
Magnetic Metal Detector Robot
Overview
The Magnetic Metal Detector Robot is designed to autonomously detect ferromagnetic metals, navigate to their location, pick them up, and store them in a dedicated compartment. This project integrates advanced robotics concepts using ROS 2 and micro-ROS, combining precise hardware control with efficient software communication systems.

Features
Ferromagnetic Metal Detection: Identifies metals that can be magnetized using a sensitive magnetic sensor.
Autonomous Navigation: Uses decision-making logic to move towards the detected metal.
Object Handling: A servo motor picks up the detected object and places it in a storage compartment.
Distributed Control System: Implements ROS 2 nodes for modular and scalable system design.
micro-ROS Integration: Leverages ESP controllers to seamlessly communicate within the ROS 2 ecosystem.
Custom Power Management: A custom-designed PCB ensures stable power delivery to all components.
Hardware Components
Key Components
ESP Controller: Handles micro-ROS communication and robot control.
Magnetic Sensor: Detects the presence of ferromagnetic metals.
Motor Driver (L298): Controls the DC motors for navigation.
Servo Motor: Picks up and stores the detected metal objects.
Power Distribution Board (PCB): Custom-designed PCB with voltage regulators, connectors, and power status LEDs.
3D Printed Body: Designed for optimal durability and functionality.
Rubber Wheels: Provide smooth movement and stability.
Miscellaneous Components: Wires, connectors, and resistors for wiring and assembly.
(Add pictures of the components here.)

ROS and micro-ROS Communication
The system utilizes ROS 2 for high-level control and micro-ROS for low-level communication between the ESP controllers and the main ROS system.

How It Works:
Sensor Data: Magnetic sensor data is published to a ROS topic by the micro-ROS client on the ESP.
Navigation: ROS nodes process the sensor data and send movement commands to the robot.
Object Handling: When the robot reaches the target, it commands the servo motor to pick up the object.
Feedback: The micro-ROS client on the ESP sends the status back to the ROS 2 system for real-time updates.
(Include a diagram of the ROS and micro-ROS communication flow here.)

PCB Design
The custom PCB integrates power regulation, connection points for components, and indicators for system health. Key features include:

Voltage Regulators: Stable power delivery for all components.
Power LEDs: Visual indicators for power status.
Magnetic Sensor Interface: Dedicated connections for the magnetic sensor.
Compact Design: Optimized layout for space efficiency.
(Add PCB images and list specific components here.)

Simulation
The physical and control aspects of the robot were simulated in MATLAB Simulink.

Simulation Includes:
Robot Movement: Models motor dynamics and trajectory planning.
Control Logic: Implements decision-making algorithms for navigation.
Object Handling: Simulates the picking and storing process.
(Insert simulation diagrams and results here.)

Flow Code
(Add a detailed flowchart or pseudocode for the robot's operation.)

Media
(Insert pictures and videos of the robot during development and operation.)

How to Use
Setup Instructions
Clone the Repository:
bash
Copy code
git clone https://github.com/yourusername/magnetic-metal-detector-robot.git
Install ROS 2: Follow the installation guide for ROS 2 Foxy or later.
Flash the ESP: Use the ESP-IDF framework to flash the provided micro-ROS code onto the ESP controller.
Run the ROS Nodes: Launch the system with:
bash
Copy code
ros2 launch robot_control.launch.py
Power the Robot: Connect the custom PCB to the power source and ensure all components are connected.
Future Improvements
Integrating advanced sensors for better accuracy.
Adding obstacle avoidance mechanisms.
Optimizing power consumption for extended operation.

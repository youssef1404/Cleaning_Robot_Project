# Cleaning_Robot_Project
# **Magnetic Metal Detector Robot**

## **Overview**  
The Magnetic Metal Detector Robot is designed to autonomously detect ferromagnetic metals, navigate to their location, pick them up, and store them in a dedicated compartment. This project integrates advanced robotics concepts using ROS 2 and micro-ROS, combining precise hardware control with efficient software communication systems.

---

## **Features**  
- **Ferromagnetic Metal Detection**: Identifies metals that can be magnetized using a sensitive magnetic sensor.  
- **Autonomous Navigation**: Uses decision-making logic to move towards the detected metal.  
- **Object Handling**: A servo motor picks up the detected object and places it in a storage compartment.  
- **Distributed Control System**: Implements ROS 2 nodes for modular and scalable system design.  
- **micro-ROS Integration**: Leverages ESP controllers to seamlessly communicate within the ROS 2 ecosystem.  
- **Custom Power Management**: A custom-designed PCB ensures stable power delivery to all components.  

---

## **Hardware Components**  

### **Key Components**  
1. **ESP Controller**: Handles micro-ROS communication and robot control.  
2. **Magnetic Sensor**: Detects the presence of ferromagnetic metals.  
3. **Motor Driver (L298)**: Controls the DC motors for navigation.  
4. **Servo Motor**: Picks up and stores the detected metal objects.  
5. **Power Distribution Board (PCB)**: Custom-designed PCB with voltage regulators, connectors, and power status LEDs.  
6. **3D Printed Body**: Designed for optimal durability and functionality.  
7. **Rubber Wheels**: Provide smooth movement and stability.  
8. **Miscellaneous Components**: Wires, connectors, and resistors for wiring and assembly.  

---

## **ROS and micro-ROS Communication**  

The system utilizes ROS 2 for high-level control and micro-ROS for low-level communication between the ESP controllers and the main ROS system.  

### **How It Works:**  
1. **Sensor Data**: Magnetic sensor data is published to a ROS topic by the micro-ROS client on the ESP.  
2. **Navigation**: ROS nodes process the sensor data and send movement commands to the robot.  
3. **Object Handling**: When the robot reaches the target, it commands the servo motor to pick up the object.  
4. **Feedback**: The micro-ROS client on the ESP sends the status back to the ROS 2 system for real-time updates.  

---

## **PCB Design**  

The custom PCB integrates power regulation, connection points for components, and indicators for system health.  

### **Key Features:**  
- **Voltage Regulators**: Stable power delivery for all components.  
- **Power LEDs**: Visual indicators for power status.  
- **Magnetic Sensor Interface**: Dedicated connections for the magnetic sensor.  
- **Compact Design**: Optimized layout for space efficiency.  


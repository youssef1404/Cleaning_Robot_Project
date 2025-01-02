#include <Timer.h>
#include "Config.h"
#include "pid_controller.h"
#include "eInterrupt.h"
#include "l298n.h"
#include "driver.h"
#include "mechServo.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <IPAddress.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int8.h>

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_publisher_t pid_output_publisher;
rcl_publisher_t speed_feedback_publisher;
rcl_publisher_t counts_publisher;
rcl_subscription_t speed_setpoint_subscriber;
rcl_subscription_t pid_parameters_subscriber;
rcl_subscription_t key_input_subscriber;

std_msgs__msg__Float32MultiArray pid_output_msg;
std_msgs__msg__Float32MultiArray speed_feedback_msg;
std_msgs__msg__Float32MultiArray counts_msg;
std_msgs__msg__Float32MultiArray speed_setpoint_msg;
std_msgs__msg__Float32MultiArray pid_parameters_msg;
std_msgs__msg__Int8 key_msg;

Timer timer;

// Function declarations
void update_encoder();
void speed_control();
void publish_readings();
void speed_setpoint_callback(const void *msgin);
void pid_parameters_callback(const void *msgin);
void key_input_callback(const void *msgin);
void Motor0_ISR_EncoderA();
void Motor0_ISR_EncoderB();
void Motor1_ISR_EncoderA();
void Motor1_ISR_EncoderB();
void create_entities();
void printWifiStatus();
void handleKeys(uint8_t key);

// Wi-Fi credentials
const char* ssid = "Araby";
const char* password = "AhmedAraby";

// Micro-ROS agent settings
const size_t agent_port = 8888;
IPAddress agent_ip(192, 168, 107, 223);

WebServer server(80);

// Creating PID objects
PIDController PID[]{PIDController(kp0, ki0, kd0, OUTPUTLIMITS, DEADZONE),
                    PIDController(kp1, ki1, kd1, OUTPUTLIMITS, DEADZONE)};

// Creating Encoder objects
eInterrupt Interrupt[]{eInterrupt(pin_A1, pin_B1, RESOLUTION),
                      eInterrupt(pin_A2, pin_B2, RESOLUTION)};

// Creating motor driver objects
L298N l298n[]{L298N(enable_pin_1, input1_1, input2_1),
              L298N(enable_pin_2, input1_2, input2_2)};
MotorDriver driver;
MechServo servo;

float counts_data[] = {0,0};
float setpoint[] = {1,1};
float speed_feedback[] = {0,0};
float pid_output[] = {0,0};
float pid_parameters[] = {kp0,ki0,kd0,kp1,ki1,kd1};

void apiMoveForward() {
  Serial.println("Starting Moving Forward...");
  driver.moveForward();
  server.send(200, "application/json", "{\"status\":\"moving forward\"}");
}

void apiMoveBackward() {
  Serial.println("Starting Moving Backward...");
  driver.moveBackward();
  server.send(200, "application/json", "{\"status\":\"moving backward\"}");
}

void apiMoveRight() {
  Serial.println("Starting Moving Right...");
  driver.rotateRight();
  server.send(200, "application/json", "{\"status\":\"moving Right\"}");
}

void apiMoveLeft() {
  Serial.println("Starting Moving Left...");
  driver.rotateLeft();
  server.send(200, "application/json", "{\"status\":\"moving Left\"}");
}

void apiMoveStop() {
  Serial.println("Starting Moving Left...");
  driver.stop();
  server.send(200, "application/json", "{\"status\":\"Stop\"}");
}

void apiEnableMagnet() {
  Serial.println("Enable Magnet...");
  digitalWrite(MAGNET_PIN, HIGH);
  server.send(200, "application/json", "{\"status\":\"Enable Magnet\"}");
}

void apiDisableMagnet() {
  Serial.println("Disable Magnet...");
  digitalWrite(MAGNET_PIN, LOW);
  server.send(200, "application/json", "{\"status\":\"Disable Magnet\"}");
}

void apiUpServo() {
  Serial.println("Up Servo...");
  servo.move(180);
  digitalWrite(MAGNET_PIN, LOW);
  server.send(200, "application/json", "{\"status\":\"Up Servo\"}");
}

void apiDownServo() {
  Serial.println("Down Servo...");
  servo.move(15);
  server.send(200, "application/json", "{\"status\":\"Down Servo\"}");
}

void apiChangeSpeed() {
  if (server.hasArg("speed")) {
    Serial.println("Changing Motor Speed...");
    int percentage = server.arg("speed").toInt();
    int speed = min_speed + (percentage * (max_speed - min_speed) / 100); 
    Serial.println("New Speed to be set: ");
    Serial.print(speed);
    driver.setSpeed(speed);
    server.send(200, "application/json", "{\"status\":\"Speed Changed\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"Speed is requied!\"}");
  }
}

void setup_routing() {
  server.on("/move-forward", HTTP_GET, apiMoveForward);
  server.on("/move-backward", HTTP_GET, apiMoveBackward);
  server.on("/move-right", HTTP_GET, apiMoveRight);
  server.on("/move-left", HTTP_GET, apiMoveLeft);
  server.on("/stop", HTTP_GET, apiMoveStop);
  server.on("/enable-magnet", HTTP_GET, apiEnableMagnet);
  server.on("/disable-magnet", HTTP_GET, apiDisableMagnet);
  server.on("/up-servo", HTTP_GET, apiUpServo);
  server.on("/down-servo", HTTP_GET, apiDownServo);
  server.on("/change-speed", HTTP_POST, apiChangeSpeed);
  server.begin();
}

void setup() {
  Serial.begin(115200);

  // Wi-Fi setup and Micro-ROS transport initialization
  char ip_buffer[16];
  snprintf(ip_buffer, sizeof(ip_buffer), "%s", agent_ip.toString().c_str());
  
  // Set Micro-ROS WiFi transports
  set_microros_wifi_transports((char*)ssid, (char*)password, ip_buffer, agent_port); 
  printWifiStatus();
  
  setup_routing();

  // Servo and motor initialization
  servo.init();
  driver.initialMotors();

  // Initialize Micro-ROS node and entities
  create_entities();

  // Run the encoder update loop at the required interval
  timer.every(TIME_FREQ, update_encoder);
}

void loop() {
  // Handle web server requests
  server.handleClient();
  
  // Handle Micro-ROS executor tasks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
}

void update_encoder(){
  counts_data[0] = Interrupt[0].encoder_counts;
  counts_data[1] = Interrupt[1].encoder_counts;
  speed_feedback[0] = Interrupt[0].calculate_speed();
  speed_feedback[1] = Interrupt[1].calculate_speed();
}

void speed_control(){
  PID[0].setParameters(pid_parameters[0], pid_parameters[1], pid_parameters[2]);
  PID[1].setParameters(pid_parameters[3], pid_parameters[4], pid_parameters[5]);

  PID[0].setSetpoint(setpoint[0]);
  PID[1].setSetpoint(setpoint[1]);

  pid_output[0] = PID[0].calculateOutput(speed_feedback[0]);
  pid_output[1] = PID[1].calculateOutput(speed_feedback[1]);

  l298n[0].set_speed(pid_output[0]);
  l298n[1].set_speed(pid_output[1]);
  l298n[0].set_direction(pid_output[0]);
  l298n[1].set_direction(pid_output[1]);

  l298n[0].control_speed();
  l298n[1].control_speed();
}

void publish_readings() {
  pid_output_msg.data.size = 2;
  speed_feedback_msg.data.size = 2;
  counts_msg.data.size = 2;

  pid_output_msg.data.data = pid_output;
  speed_feedback_msg.data.data = speed_feedback;
  counts_msg.data.data = counts_data;
  
  rcl_publish(&pid_output_publisher, &pid_output_msg, NULL);
  rcl_publish(&speed_feedback_publisher, &speed_feedback_msg, NULL);
  rcl_publish(&counts_publisher, &counts_msg, NULL);
  
  Serial.println("Encoder counts:");
  Serial.println(counts_data[0]);
  Serial.println(counts_data[1]);
}

void speed_setpoint_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  setpoint[0] = msg->data.data[0];
  setpoint[1] = msg->data.data[1];
  Serial.println("Setpoint 1: ");
  Serial.println(setpoint[0]);
  Serial.println("Setpoint 2: ");
  Serial.println(setpoint[1]);
}

void pid_parameters_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  pid_parameters[0] = msg->data.data[0];
  pid_parameters[1] = msg->data.data[1];
  pid_parameters[2] = msg->data.data[2];
  pid_parameters[3] = msg->data.data[3];
  pid_parameters[4] = msg->data.data[4];
  pid_parameters[5] = msg->data.data[5];
}

void key_input_callback(const void *msgin) {
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
  int8_t key = msg->data;
  Serial.println("Keyboard number: ");
  Serial.println(key);
  handleKeys(key);
}

void Motor0_ISR_EncoderA() {
  Interrupt[0].ISR_A_routine();
}

void Motor0_ISR_EncoderB() {
  Interrupt[0].ISR_B_routine();
}

void Motor1_ISR_EncoderA() {
  Interrupt[1].ISR_A_routine();
}

void Motor1_ISR_EncoderB() {
  Interrupt[1].ISR_B_routine();
}

void printWifiStatus() {
  Serial.print("Wi-Fi Status: ");
  Serial.println(WiFi.status());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal strength (RSSI): ");
  Serial.println(WiFi.RSSI());
}

void create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "movement", "", &support);

  rclc_publisher_init_default(
    &speed_feedback_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "speed_feedback");

  rclc_publisher_init_default(
    &pid_output_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "pid_output"); 

  rclc_publisher_init_default(
    &counts_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "counts"); 

  rclc_subscription_init_default(
    &speed_setpoint_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "speed_setpoint");

  rclc_subscription_init_default(
    &pid_parameters_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "pid_parameters");

  rclc_subscription_init_default(
    &key_input_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/key_input");

  speed_setpoint_msg.data.capacity = 2;
  pid_parameters_msg.data.capacity = 6;

  speed_setpoint_msg.data.data = (float_t*) malloc(speed_setpoint_msg.data.capacity * sizeof(float_t));
  pid_parameters_msg.data.data = (float_t*) malloc(pid_parameters_msg.data.capacity * sizeof(float_t));

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &speed_setpoint_subscriber, &speed_setpoint_msg, &speed_setpoint_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &pid_parameters_subscriber, &pid_parameters_msg, &pid_parameters_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &key_input_subscriber, &key_msg, &key_input_callback, ON_NEW_DATA);
}

void handleKeys(uint8_t key){
  switch(key) {
    case 5: // Forward
      driver.moveForward();
      break;
    case 6: // Backward
      driver.moveBackward();
      break;
    case 8: // Left turn
      driver.rotateLeft();
      break;
    case 7: // Right turn
      driver.rotateRight();
      break;
    case 9:
      driver.stop();
      break;
    case 1: // up servo
      servo.move(180);
      break;
    case 2: // down servo
      servo.move(15);
      break;
    case 3: // turn on the magnet
      digitalWrite(MAGNET_PIN, HIGH);
      break;
    case 4: // turn off the magnet
      digitalWrite(MAGNET_PIN, LOW);
      break;
    case 10: // turn off the magnet
      driver.increaseSpeed();
      break;
    case 11: // turn off the magnet
      driver.decreaseSpeed();
      break;
    default:
      break;
  }
}


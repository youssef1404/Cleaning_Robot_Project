#include <Arduino.h>
#include <Timer.h>
#include "Config.h"
#include "pid_controller.h"
#include "interrupt.h"
#include "l298n.h"

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi__array.h>
Timer timer;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rcl_timer_t timer;

std_msgs__msg__Float32MultiArray encoder_speed;
std_msgs__msg__Float32MultiArray counts;
std_msgs__msg__Float32MultiArray pid_output;
std_msgs__msg__Float32MultiArray pid_parameters;

rcl_publisher_t encoder_speed_publisher;
rcl_publisher_t counts_publisher;
rcl_publisher_t pid_publisher;

rcl_subscription_t SetPoint_subscriber;
rcl_subscription_t parameters_subscriber;

float counts_data[2];
float setpoint[2];


void set_setpoint(const std_msgs::Float32MultiArray &msg);
void set_parameters(const std_msgs::Float32MultiArray& msg);

//creating PID objects
PIDController PID[]{PIDController(kp0,ki0,kd0,OUTPUTLIMITS,DEADZONE),
                    PIDController(kp1,ki1,kd1,OUTPUTLIMITS,DEADZONE)};

//creating Encoder objects
Interrupt interrupt[]{Interrupt(pin_A1,pin_B1,RESOLUTION),
                      Interrupt(pin_A2,pin_B2,RESOLUTION)};

//creating motor driver objects
L298N l298n[]{
             L298N(enable_pin_1,input1_1,input2_1),
             L298N(enable_pin_2,input1_2,input2_2)
};

float encoder_feedback[] = {0,0};
float pid[] = {0,0};

void setup(){
    interrupt[0].Init();
    interrupt[1].Init();

    l298n[0].driver_init();
    l298n[1].driver_init();

    analogWriteResolution(16);
    timer.every(TIME_FREQ, update_encoder);

    attachInterrupt(digitalPinToInterrupt(interrupt[0].pin_A), Motor0_ISR_EncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interrupt[0].pin_B), Motor0_ISR_EncoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interrupt[1].pin_A), Motor1_ISR_EncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interrupt[1].pin_B), Motor1_ISR_EncoderB, CHANGE);

    set_microros_transports();
    rcl_allocator_t allocator = rcl_get_default_allocator();  // Use the default allocator
    rclc_support_init(&support, 0, NULL, &allocator);  // Pass the allocator as a pointer

    rclc_publisher_init_default(&encoder_speed_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "encoder_speed");
    rclc_publisher_init_default(&counts_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "counts");
    rclc_publisher_init_default(&pid_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "pid_pub");

    rclc_subscription_init_default(&SetPoint_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "Setpoint");
    rclc_subscription_init_default(&parameters_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "Setparam");

    rclc_executor_add_subscription(&executor, &SetPoint_subscriber, &setpoint, &setpointCallback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &parameters_subscriber, &setParameters, &setParametersCallback, ON_NEW_DATA); 

  // Set up length of arrays
  speed_msg.data.data = encoder_feedback;
  speed_msg.data.size = 2;
  
  counts_msg.data.data = counts_data;
  counts_msg.data.size = 2;

  pid_output_msg.data.data = pid;
  pid_output_msg.data.size = 2;
}

void loop(){
  timer.update();


}

void update_encoder(){
    encoder_feedback[0] = interrupt[0].calculate_speed();
    encoder_feedback[1] = interrupt[1].calculate_speed();
}

void speed_controll(){

    pid[0].setSetpoint(setpoint[0]);
    pid[1].setSetpoint(setpoint[1]);

    pid[0]=PID[0].calculateOutput(encoder_feedback[0]);
    pid[1]=PID[1].calculateOutput(encoder_feedback[1]);

    l298n[0].set_speed(pid[0]);
    l298n[1].set_speed(pid[1]);
    l298n[0].set_direction(pid[0]);
    l298n[1].set_direction(pid[1]);

    l298n[0].control_speed();
    l298n[1].control_speed();


    speed_msg.data.data = encoder_feedback[0];
    speed_msg.data.data[1] = encoder_feedback[1];


    counts_msg.data.data[0] = interrupt[0].encoder_counts;
    counts_msg.data.data[1] = interrupt[1].encoder_counts;


   
    pid_output_msg.data.data[0] = pid[0];
    pid_output_msg.data.data[1] = pid[1];



    publish_messages();
}

void Motor0_ISR_EncoderA()
{
  interrupt[0].ISR_A_routine();
}

void Motor0_ISR_EncoderB()
{
  interrupt[0].ISR_B_routine();
}

void Motor1_ISR_EncoderA()
{
  interrupt[1].ISR_A_routine();
}

void Motor1_ISR_EncoderB()
{
  interrupt[1].ISR_B_routine();
}

void set_parameters(const std_msgs::Float32MultiArray& msg) {
      // Update PID parameters
      PID[0].setParameters(msg.data[0], msg.data[1], msg.data[2]);
      PID[1].setParameters(msg.data[3], msg.data[4], msg.data[5]);
}

void setpointCallback(const void *msg_in){
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msg_in;
  
    setpoint[0] = msg->data.data[i];
    setpoint[1] = msg->data.data[i];
  
}

void setParametersCallback(const void *msg_in){
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msg_in;
    

  pid[0].setParameters(msg->data.data[0], msg->data.data[1],  msg->data.data[2]);
  pid[1].setParameters(msg->data.data[3], msg->data.data[4],  msg->data.data[5]);

}


void publish_messages(){
    rcl_ret_t ret = rcl_publish(&encoder_speed_publisher, &speed_msg, NULL);
    ret = rcl_publish(&counts_publisher, &counts_msg, NULL);
    ret = rcl_publish(&pid_publisher , &pid_output_msg, NULL);
}
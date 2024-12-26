#ifndef _ROSCOMM_H_
#define _ROSCOMM_H_

#include <micro_ros_platformio.h>
#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <IPAddress.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>


// Wi-Fi credentials
#define WIFI_SSID "Honor 50"       
#define WIFI_PASSWORD "12345678" 
#define AGENT_IP "192.168.5.203"           
#define AGENT_PORT 8888 

#if defined(LED_BUILTIN)
  #define LED_PIN LED_BUILTIN
#else
  #define LED_PIN 13
#endif

// WIFI
extern IPAddress agent_ip;
extern size_t agent_port;
extern char ssid[];
extern char psk[];

enum States {
WAITING_AGENT, // Pings the agent.
AGENT_AVAILABLE, // Creates ROS 2 entities.
AGENT_CONNECTED, // Spins the executor.
AGENT_DISCONNECTED // Cleans up and resets.
};

class RosComm
{
    private:
        // Wifi status
        IPAddress agent_ip;
        size_t agent_port;
        const char *ssid;
        const char *psk;

        States state;

        // entities
        rclc_executor_t executor;
        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;
        const char *node_name;
        rcl_timer_t timer;

        // publishers
        // static rcl_publisher_t ultra_pub; // ultrasonic data
        // const char *ultra_topic_name;
        // static rcl_publisher_t motor_feed_pub; 
        // const char *motor_feed_topic_name;

        // subscirbers
        rcl_subscription_t my_sub;
        const char *motion_topic_name;

        // static void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
        static void my_subscriber_callback(const void * msgin);
        void destroy();
        bool create_entities();

        float distance;
        int8_t keyVal;

    public:
        bool sub_new_data;
        static std_msgs__msg__Int8* key_msg;
        // static std_msgs__msg__Float32* ultra_msg;

        RosComm();
        bool initialize();
        void loop();
        // void updateUltraMsg();
        // float getDistance();
        void updateKeyboardValue();
        int8_t getkeyboardValue();
        void printWifiStatus();
};

#endif
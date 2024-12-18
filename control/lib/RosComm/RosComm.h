#ifndef _ROSCOMM_H_
#define _ROSCOMM_H_

#include <micro_ros_platformio.h>
#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <IPAddress.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

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
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
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
        static rcl_publisher_t ultra_pub; // ultrasonic data
        const char *ultra_topic_name;
        static rcl_publisher_t motor_feed_pub; 
        const char *motor_feed_topic_name;

        // subscirbers
        rcl_subscription_t my_sub;
        const char *motion_topic_name;

        static void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
        static void my_subscriber_callback(const void * msgin);
        void destroy();
        bool create_entities();

        float distance;

    public:
        bool sub_new_data;
        static std_msgs__msg__Int32 key_msg;
        static std_msgs__msg__Float32 ultra_msg;

        RosComm();
        void initialize();
        void loop();
        void printWifiData();
        void printCurrentNet();
        void updateUltraMsg();
        float getDistance();

};


#endif
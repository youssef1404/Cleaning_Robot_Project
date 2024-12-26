#ifndef ROSCOMM_H
#define ROSCOMM_H

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <WiFi.h>

#define LED_PIN 2
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"
#define AGENT_IP "192.168.1.18"
#define AGENT_PORT 8888

class RosComm
{
public:
    RosComm();
    bool initialize();
    void loop();
    int8_t getkeyboardValue();
    void printWifiStatus();

private:
    enum CommState
    {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    };

    bool create_entities();
    void destroy();
    void updateKeyboardValue();
    bool setupWiFiConnection();
    static void my_subscriber_callback(const void *msgin);

    const char *node_name;
    const char *motion_topic_name;
    IPAddress agent_ip;
    uint16_t agent_port;
    const char *ssid;
    const char *psk;
    int8_t keyVal;

    rcl_subscription_t my_sub;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rclc_executor_t executor;
    CommState state;

    static std_msgs__msg__Int8 *key_msg;

    // Connection management
    int connection_attempts;
    unsigned long last_reconnect_attempt;
};

#endif // ROSCOMM_H
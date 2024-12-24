#ifndef _ROSCOMM_H_
#define _ROSCOMM_H_

#include <micro_ros_platformio.h>
#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <IPAddress.h>
#include "ros_config.h"
#include "ros_types.h"

class RosComm
{
private:
    // Network configuration
    NetworkConfig network_config;

    // ROS entities
    RosEntities entities;

    // State management
    States state;

    // Data storage
    float distance;
    int keyVal;

    // Internal methods
    bool create_entities();
    void destroy();
    static void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
    static void subscriber_callback(const void *msgin);

public:
    // Static message holders
    static std_msgs__msg__Int32 key_msg;
    static std_msgs__msg__Float32 ultra_msg;
    static rcl_publisher_t ultra_pub;
    static rcl_publisher_t motor_feed_pub;

    bool sub_new_data;

    RosComm();
    void initialize();
    void loop();

    // Utility methods
    void printWifiData();
    void printCurrentNet();

    // Data access methods
    void updateUltraMsg();
    float getDistance() const { return distance; }
    void updateKeyboardValue();
    int getKeyboardValue() const { return keyVal; }
    States getState() const { return state; }
};

#endif
```cpp
#ifndef _ROS_TYPES_H_
#define _ROS_TYPES_H_

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

    enum States {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    };

struct NetworkConfig
{
    IPAddress agent_ip;
    size_t agent_port;
    const char *ssid;
    const char *psk;
};

struct RosEntities
{
    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    const char *node_name;
    rcl_timer_t timer;
    rcl_subscription_t subscriber;
    const char *ultra_topic_name;
    const char *motor_feed_topic_name;
    const char *motion_topic_name;
};

#endif
```
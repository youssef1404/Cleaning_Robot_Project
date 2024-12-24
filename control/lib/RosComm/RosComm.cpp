#include "RosComm.h"

rcl_publisher_t RosComm::ultra_pub;
rcl_publisher_t RosComm::motor_feed_pub;
std_msgs__msg__Int32 RosComm::key_msg;
std_msgs__msg__Float32 RosComm::ultra_msg;

RosComm::RosComm()
{
    entities.node_name = "esp32";
    entities.ultra_topic_name = "/distance";
    entities.motion_topic_name = "/motion";

    network_config.agent_ip = IPAddress(192, 168, 1, 18);
    network_config.agent_port = AGENT_PORT;
    network_config.ssid = WIFI_SSID;
    network_config.psk = WIFI_PASSWORD;

    entities.subscriber = rcl_get_zero_initialized_subscription();
    sub_new_data = false;
}

void RosComm::initialize()
{
    set_microros_wifi_transports(
        network_config.ssid,
        network_config.psk,
        network_config.agent_ip,
        network_config.agent_port);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(2000);

    state = WAITING_AGENT;
}

bool RosComm::create_entities()
{
    // Initialize allocator
    entities.allocator = rcl_get_default_allocator();

    // Initialize support
    RCCHECK(rclc_support_init(&entities.support, 0, NULL, &entities.allocator));

    // Create node
    RCCHECK(rclc_node_init_default(
        &entities.node,
        entities.node_name,
        "",
        &entities.support));

    // Initialize subscriber
    RCCHECK(rclc_subscription_init_default(
        &entities.subscriber,
        &entities.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        entities.motion_topic_name));

    // Initialize publishers
    RCCHECK(rclc_publisher_init_best_effort(
        &ultra_pub,
        &entities.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        entities.ultra_topic_name));

    // Initialize timer
    RCCHECK(rclc_timer_init_default(
        &entities.timer,
        &entities.support,
        RCL_MS_TO_NS(100),
        timer_callback));

    // Initialize executor
    RCCHECK(rclc_executor_init(
        &entities.executor,
        &entities.support.context,
        2,
        &entities.allocator));

    RCCHECK(rclc_executor_add_subscription(
        &entities.executor,
        &entities.subscriber,
        &key_msg,
        subscriber_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(
        &entities.executor,
        &entities.timer));

    return true;
}

void RosComm::timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer != NULL)
    {
        std::ignore = rcl_publish(&ultra_pub, &ultra_msg, NULL);
    }
}

void RosComm::subscriber_callback(const void *msgin)
{
    if (msgin != NULL)
    {
        key_msg = *(std_msgs__msg__Int32 *)msgin;
    }
}

void RosComm::loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500,
                           state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                       ? AGENT_AVAILABLE
                                       : WAITING_AGENT;);
        break;

    case AGENT_AVAILABLE:
        state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroy();
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200,
                           state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                       ? AGENT_CONNECTED
                                       : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&entities.executor, RCL_MS_TO_NS(100));
        }
        break;

    case AGENT_DISCONNECTED:
        destroy();
        state = WAITING_AGENT;
        break;
    }

    digitalWrite(LED_PIN, state == AGENT_CONNECTED ? HIGH : LOW);

    updateKeyboardValue();
    updateUltraMsg();
}

void RosComm::destroy()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&entities.support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    std::ignore = rcl_publisher_fini(&ultra_pub, &entities.node);
    std::ignore = rcl_subscription_fini(&entities.subscriber, &entities.node);
    std::ignore = rcl_timer_fini(&entities.timer);

    rclc_executor_fini(&entities.executor);
    std::ignore = rcl_node_fini(&entities.node);
    rclc_support_fini(&entities.support);

    std_msgs__msg__Int32__fini(&key_msg);
    std_msgs__msg__Float32__fini(&ultra_msg);
}

// Utility methods implementation...
void RosComm::printWifiData()
{
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void RosComm::printCurrentNet()
{
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("Signal strength (RSSI): ");
    Serial.println(WiFi.RSSI());
}

void RosComm::updateUltraMsg()
{
    distance = ultra_msg.data;
}

void RosComm::updateKeyboardValue()
{
    if (state == AGENT_CONNECTED)
    {
        keyVal = key_msg.data;
    }
}
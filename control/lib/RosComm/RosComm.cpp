#include "RosComm.h"

// rcl_publisher_t RosComm::ultra_pub;
// rcl_publisher_t RosComm::motor_feed_pub;
std_msgs__msg__Int8* RosComm::key_msg = NULL;
// std_msgs__msg__Float32* RosComm::ultra_msg = NULL;

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }

#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

RosComm::RosComm(): node_name("esp32"),
                    // ultra_topic_name("/distance"),
					motion_topic_name("/key_input"),
                    agent_ip(192, 168, 1, 18),
                    agent_port(AGENT_PORT),
                    ssid(WIFI_SSID),
                    psk(WIFI_PASSWORD),
                    keyVal(0),
                    distance(0.0f)
{
    this->my_sub = rcl_get_zero_initialized_subscription();
    this->state = WAITING_AGENT;
}

bool RosComm::initialize() {
    Serial.println("Starting initialization...");
    
    destroy();  // Make sure we start clean
    
    // Serial.printf("Free heap before allocation: %d\n", ESP.getFreeHeap());
    
    // Add delay to ensure serial prints are visible
    delay(1000);

    set_microros_wifi_transports(WIFI_SSID, 
                                WIFI_PASSWORD, 
                                AGENT_IP, 
                                AGENT_PORT);
	// printWifiStatus();



    // RosComm::ultra_msg->data = 0.0f;

    Serial.println("Setting up LED...");
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    Serial.println("Initialization complete!");
    this->state = WAITING_AGENT;
    return true;
}

bool RosComm::create_entities() {
    // create allocator
    this->allocator = rcl_get_default_allocator();

    // create init_options
    if (RCL_RET_OK != rclc_support_init(&this->support, 0, NULL, &this->allocator)) {
        Serial.println("Failed to initialize support");
        return false;
    }

    // create node
    if (RCL_RET_OK != rclc_node_init_default(&this->node, this->node_name, "", &this->support)) {
        Serial.println("Failed to initialize node");
        return false;
    }

    // create subscriber
    if (RCL_RET_OK != rclc_subscription_init_default(
        &this->my_sub,
        &this->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        this->motion_topic_name)) {
        Serial.println("Failed to create subscriber");
        return false;
    }

    // // create publisher
    // if (RCL_RET_OK != rclc_publisher_init_best_effort(
    //     &this->ultra_pub,
    //     &this->node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    //     this->ultra_topic_name)) {
    //     Serial.println("Failed to create publisher");
    //     return false;
    // }

    // // create timer
    // if (RCL_RET_OK != rclc_timer_init_default2(
    //     &timer,
    //     &support,
    //     RCL_MS_TO_NS(1000),
    //     timer_callback,
    //     true)) {
    //     Serial.println("Failed to create timer");
    //     return false;
    // }

    // Initialize message structures with checks
    Serial.println("Creating message structures...");
    RosComm::key_msg = (std_msgs__msg__Int8*)malloc(sizeof(std_msgs__msg__Int8));
    if (RosComm::key_msg == NULL) {
        Serial.println("Failed to create key_msg");
        return false;
    }
    Serial.println("Initializing message data...");
    RosComm::key_msg->data = 0;


    if (RCL_RET_OK != rclc_executor_init(&this->executor, &this->support.context, 2, &this->allocator)) {
        Serial.println("Failed to initialize executor");
        return false;
    }

    if (RCL_RET_OK != rclc_executor_add_subscription(
        &this->executor,
        &this->my_sub,
        &RosComm::key_msg,
        &RosComm::my_subscriber_callback,
        ON_NEW_DATA)) {
        Serial.println("Failed to add subscription to executor");
        return false;
    }

    if (RCL_RET_OK != rclc_executor_add_timer(&this->executor, &this->timer)) {
        Serial.println("Failed to add timer to executor");
        return false;
    }

    return true;
}

// void RosComm::timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//     RCLC_UNUSED(last_call_time);
//     if (timer != NULL && RosComm::ultra_msg != NULL) {
//         // Add debug print for publishing
//         Serial.println(RosComm::ultra_msg->data);
//         std::ignore = rcl_publish(&RosComm::ultra_pub, RosComm::ultra_msg, NULL);
//     }
// }

void RosComm::my_subscriber_callback(const void *msgin) {
    Serial.println("Subscriber callback entered");
    
    if (msgin == NULL) {
        Serial.println("Error: Received NULL message");
        return;
    }
    
    if (RosComm::key_msg == NULL) {
        Serial.println("Error: key_msg is NULL");
        return;
    }

    const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
    Serial.printf("Received message data: %d\n", msg->data);
    RosComm::key_msg->data = msg->data;
}

void RosComm::loop() {
    static unsigned long lastPrint = 0;
    unsigned long now = millis();
    
    // Print state every second
    if (now - lastPrint >= 1000) {
        Serial.printf("Current state: %d, Free heap: %d\n", state, ESP.getFreeHeap());
        lastPrint = now;
    }

    // Add null checks before state machine
    if (RosComm::key_msg == NULL ) {
        Serial.println("Error: Message structures are NULL, reinitializing...");
        if (initialize()) {
            Serial.println("Reinitialization successful");
        } else {
            Serial.println("Reinitialization failed");
            delay(1000);
            return;
        }
    }

    switch (state) {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, {
            bool ping_ok = (RMW_RET_OK == rmw_uros_ping_agent(100, 1));
            Serial.printf("Ping agent result: %d\n", ping_ok);
            state = ping_ok ? AGENT_AVAILABLE : WAITING_AGENT;
        });
        break;

    case AGENT_AVAILABLE:
        Serial.println("Agent available, creating entities...");
        state = (true == this->create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
            Serial.println("Failed to create entities");
            this->destroy();
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, {
            bool ping_ok = (RMW_RET_OK == rmw_uros_ping_agent(500, 1));
            if (!ping_ok) {
                Serial.println("Lost connection to agent");
                state = AGENT_DISCONNECTED;
            }
        });
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&this->executor, RCL_MS_TO_NS(100));
        }
    
        break;

    case AGENT_DISCONNECTED:
        Serial.println("Agent disconnected, cleaning up...");
        this->destroy();
        state = WAITING_AGENT;
        break;
    }

    // Add delay to prevent watchdog triggers
    delay(10);


    updateKeyboardValue();
    // updateUltraMsg();
}

void RosComm::destroy() {
    if (this->state != AGENT_CONNECTED) {
        // Only clean up ROS entities if we were connected
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        
        // std::ignore = rcl_publisher_fini(&RosComm::ultra_pub, &this->node);
        std::ignore = rcl_subscription_fini(&this->my_sub, &this->node);
        std::ignore = rcl_timer_fini(&this->timer);
        rclc_executor_fini(&this->executor);
        std::ignore = rcl_node_fini(&this->node);
        rclc_support_fini(&this->support);
    }
    
    // Always clean up message structures
    if (RosComm::key_msg != NULL) {
        std_msgs__msg__Int8__destroy(RosComm::key_msg);
        RosComm::key_msg = NULL;
    }
    // if (RosComm::ultra_msg != NULL) {
    //     std_msgs__msg__Float32__destroy(RosComm::ultra_msg);
    //     RosComm::ultra_msg = NULL;
    // }
}

// void RosComm::updateUltraMsg() {
//     if (RosComm::ultra_msg != NULL) {
//         this->distance = RosComm::ultra_msg->data;
//         // Add debug print for ultrasonic updates
//         EXECUTE_EVERY_N_MS(1000, {  // Print every second to avoid flooding Serial
//             Serial.print("Current distance: ");
//             Serial.println(this->distance);
//         });
//     }
// }

// float RosComm::getDistance() {
//     return this->distance;
// }

void RosComm::updateKeyboardValue() { 
    if (state == AGENT_CONNECTED && RosComm::key_msg != NULL && RosComm::key_msg->data != 0) {
        this->keyVal = RosComm::key_msg->data;
        // Add debug print for keyboard value updates
        // Serial.print("Updated keyboard value: ");
        // Serial.println(this->keyVal);
    }
    else this->keyVal = 0;
}

int8_t RosComm::getkeyboardValue() {
	if (state == AGENT_CONNECTED)
    	return this->keyVal;
	return 0;
}

void RosComm::printWifiStatus() {
    Serial.print("WiFi Status: ");
    Serial.println(WiFi.status());
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength (RSSI): ");
    Serial.println(WiFi.RSSI());
}
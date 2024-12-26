#include "RosComm.h"

std_msgs__msg__Int8 *RosComm::key_msg = NULL;

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

RosComm::RosComm() : node_name("esp32"),
                     motion_topic_name("/key_input"),
                     agent_ip(192, 168, 1, 18),
                     agent_port(AGENT_PORT),
                     ssid(WIFI_SSID),
                     psk(WIFI_PASSWORD),
                     keyVal(0)
{
    this->my_sub = rcl_get_zero_initialized_subscription();
    this->state = WAITING_AGENT;
    this->connection_attempts = 0;
    this->last_reconnect_attempt = 0;
}

bool RosComm::initialize()
{
    Serial.println("Starting initialization...");

    destroy(); // Make sure we start clean

    delay(1000);

    // Initialize WiFi with retry mechanism
    if (!setupWiFiConnection())
    {
        Serial.println("Failed to establish WiFi connection");
        return false;
    }

    set_microros_wifi_transports(WIFI_SSID,
                                 WIFI_PASSWORD,
                                 AGENT_IP,
                                 AGENT_PORT);

    Serial.println("Creating message structures...");
    RosComm::key_msg = std_msgs__msg__Int8__create();
    if (RosComm::key_msg == NULL)
    {
        Serial.println("Failed to create key_msg");
        return false;
    }

    RosComm::key_msg->data = 0;

    Serial.println("Setting up LED...");
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Serial.println("Initialization complete!");
    this->state = WAITING_AGENT;
    return true;
}

bool RosComm::setupWiFiConnection()
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("\nWiFi connection failed!");
        return false;
    }

    Serial.println("\nWiFi connected!");
    printWifiStatus();
    return true;
}

bool RosComm::create_entities()
{
    this->allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));
    RCCHECK(rclc_node_init_default(&this->node, this->node_name, "", &this->support));

    // Create subscriber with more robust error checking
    rcl_ret_t sub_ret = rclc_subscription_init_default(
        &this->my_sub,
        &this->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        this->motion_topic_name);

    if (sub_ret != RCL_RET_OK)
    {
        Serial.printf("Failed to create subscriber: %d\n", sub_ret);
        return false;
    }

    // Initialize executor with proper error checking
    RCCHECK(rclc_executor_init(&this->executor, &this->support.context, 1, &this->allocator));

    RCCHECK(rclc_executor_add_subscription(
        &this->executor,
        &this->my_sub,
        &RosComm::key_msg,
        &RosComm::my_subscriber_callback,
        ON_NEW_DATA));

    return true;
}

void RosComm::my_subscriber_callback(const void *msgin)
{
    if (msgin == NULL || RosComm::key_msg == NULL)
    {
        return;
    }

    const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
    RosComm::key_msg->data = msg->data;
    Serial.printf("Received key: %d\n", msg->data);
}

void RosComm::loop()
{
    // Check WiFi connection first
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi disconnected. Attempting to reconnect...");
        if (setupWiFiConnection())
        {
            state = WAITING_AGENT;
        }
        delay(1000);
        return;
    }

    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(1000, {
            bool ping_ok = (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)); // Increased timeout and retries
            if (ping_ok)
            {
                Serial.println("Agent found!");
                state = AGENT_AVAILABLE;
                connection_attempts = 0;
            }
            else
            {
                connection_attempts++;
                if (connection_attempts > 5)
                {
                    Serial.println("Multiple connection attempts failed. Reinitializing...");
                    initialize();
                    connection_attempts = 0;
                }
            }
        });
        break;

    case AGENT_AVAILABLE:
        Serial.println("Creating entities...");
        if (create_entities())
        {
            state = AGENT_CONNECTED;
            Serial.println("Successfully connected to agent");
        }
        else
        {
            Serial.println("Failed to create entities");
            delay(1000);
            state = WAITING_AGENT;
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(500, {
            bool ping_ok = (RMW_RET_OK == rmw_uros_ping_agent(100, 2));
            if (!ping_ok)
            {
                unsigned long current_time = millis();
                if (current_time - last_reconnect_attempt > 5000)
                { // Wait 5 seconds between reconnection attempts
                    Serial.println("Lost connection to agent. Attempting to reconnect...");
                    state = AGENT_DISCONNECTED;
                    last_reconnect_attempt = current_time;
                }
            }
        });

        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&this->executor, RCL_MS_TO_NS(100));
        }
        break;

    case AGENT_DISCONNECTED:
        Serial.println("Cleaning up and attempting to reconnect...");
        destroy();
        delay(1000); // Wait before attempting to reconnect
        state = WAITING_AGENT;
        break;
    }

    updateKeyboardValue();
    delay(10); // Small delay to prevent watchdog triggers
}

void RosComm::destroy()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_subscription_fini(&this->my_sub, &this->node);
    rclc_executor_fini(&this->executor);
    rcl_node_fini(&this->node);
    rclc_support_fini(&this->support);

    if (RosComm::key_msg != NULL)
    {
        std_msgs__msg__Int8__destroy(RosComm::key_msg);
        RosComm::key_msg = NULL;
    }
}

void RosComm::updateKeyboardValue()
{
    if (state == AGENT_CONNECTED && RosComm::key_msg != NULL && RosComm::key_msg->data != 0)
    {
        this->keyVal = RosComm::key_msg->data;
    }
}

int8_t RosComm::getkeyboardValue()
{
    if (state == AGENT_CONNECTED)
        return this->keyVal;
    return 0;
}

void RosComm::printWifiStatus()
{
    Serial.print("WiFi Status: ");
    Serial.println(WiFi.status());
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength (RSSI): ");
    Serial.println(WiFi.RSSI());
}
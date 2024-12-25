#include "RosComm.h"
#include "config.h"

rcl_publisher_t RosComm::ultra_pub;
rcl_publisher_t RosComm::motor_feed_pub;
std_msgs__msg__Int32 RosComm::key_msg;
std_msgs__msg__Float32 RosComm::ultra_msg;


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
                    ultra_topic_name("/distance"),
                    // motor_feed_topic_name("/motors"),
                    agent_ip(192, 168, 1, 18),
                    agent_port(AGENT_PORT),
                    ssid(WIFI_SSID),
                    psk(WIFI_PASSWORD)
{
    this->my_sub = rcl_get_zero_initialized_subscription();
}

RosComm::~RosComm() {
    destroy();
}

void RosComm::initialize(){
    set_microros_wifi_transports(WIFI_SSID, 
								WIFI_PASSWORD, 
								AGENT_IP, 
								AGENT_PORT);


    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(2000);

    this->state = WAITING_AGENT;
}

bool RosComm::create_entities(){
    // create allocator
	this->allocator = rcl_get_default_allocator();

	// create init_options
	rclc_support_init(&this->support, 0, NULL, &this->allocator);

	// create node
	rclc_node_init_default(&this->node, this->node_name, "", &this->support);

    // create subscriber
	rclc_subscription_init_default(
		&this->my_sub,
		&this->node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
		this->motion_topic_name);

    // create publisher
	rclc_publisher_init_best_effort(
		&this->ultra_pub,
		&this->node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		this->ultra_topic_name);

	// rclc_publisher_init_best_effort(
	// 	&this->motor_feed_pub,
	// 	&this->node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
	// 	this->motor_feed_topic_name);

    // create timer,
	const unsigned int timer_timeout = 100;

	RCCHECK(rclc_timer_init_default2(&timer, 
									&support, 
									RCL_MS_TO_NS(1000), 
									timer_callback, 
									true));


    rclc_executor_init(&this->executor, &this->support.context, 2, &this->allocator);
	rclc_executor_add_subscription(&this->executor, &this->my_sub, &RosComm::key_msg, RosComm::my_subscriber_callback, ON_NEW_DATA);
	rclc_executor_add_timer(&this->executor, &this->timer);

	return true;
}

void RosComm::timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		std::ignore = rcl_publish(&RosComm::ultra_pub, &RosComm::ultra_msg, NULL);
		// std::ignore = rcl_publish(&RosComm::motor_feed_pub, &RosComm::motor_feed, NULL);
	}
}

// Subscriber callback
void RosComm::my_subscriber_callback(const void *msgin)
{
	if (msgin != NULL)
	{
		RosComm::key_msg = *(std_msgs__msg__Int32 *)msgin; // Should i uodate key number only if msgin not null
        // update_keyboard_number();
	}
}

// Main Ros Loop
void RosComm::loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
		break;

    case AGENT_AVAILABLE:
        state = (true == this->create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
		if (state == WAITING_AGENT)
		{
			this->destroy();
		};
		break;

    case AGENT_CONNECTED:
		EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
		if (state == AGENT_CONNECTED)
		{
			rclc_executor_spin_some(&this->executor, RCL_MS_TO_NS(100));
		}
		break;

	case AGENT_DISCONNECTED:
		this->destroy();
		state = WAITING_AGENT;
		break;

    default:
        break;
    }

    if (state == AGENT_CONNECTED)
            digitalWrite(LED_PIN, HIGH);
	else
	{
		digitalWrite(LED_PIN, LOW);
		delay(100);
	}

    updateKeyboardValue();
    updateUltraMsg();
}

// Cleans up all ROS 2 entities and finalizes messages.
void RosComm::destroy()
{
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
	(void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
	std::ignore = rcl_publisher_fini(&RosComm::ultra_pub, &this->node);
	// std::ignore = rcl_publisher_fini(&RosComm::motor_feed_pub, &this->node);
	std::ignore = rcl_subscription_fini(&this->my_sub, &this->node);
	std::ignore = rcl_timer_fini(&this->timer);
	rclc_executor_fini(&this->executor);
	std::ignore = rcl_node_fini(&this->node);
	rclc_support_fini(&support);
	std_msgs__msg__Int32__fini(&RosComm::key_msg);
	// std_msgs__msg__Int16MultiArray__fini(&RosComm::motor_feed);
	std_msgs__msg__Float32__fini(&RosComm::ultra_msg);
}

void RosComm::printWifiData()
{
	// print your board's IP address:
	Serial.print("IP Address: ");
	Serial.println(WiFi.localIP());
}

void RosComm::printCurrentNet()
{
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print the received signal strength:
	Serial.print("signal strength (RSSI):");
	Serial.println(WiFi.RSSI());
}

void RosComm::updateUltraMsg()
{
    this->distance = RosComm::ultra_msg.data;
}

float RosComm::getDistance(){
    return this->distance;
}

void RosComm::updateKeyboardValue(){ 
    if (state == AGENT_CONNECTED)
        this->keyVal = RosComm::key_msg.data;
}

int RosComm::getkeyboardValue(){
    return this->keyVal;
}

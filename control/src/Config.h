#define DEADZONE 4
#define OUTPUTLIMITS 255.0
#define RESOLUTION 44
#define TIME_FREQ 30
#define dt 0.03

// PID parameters
#define kp0 1.0
#define ki0 0.1
#define kd0 0.01
#define kp1 1.0
#define ki1 0.1
#define kd1 0.01

#define pin_A1 36
#define pin_A2 34

#define pin_B1 39
#define pin_B2 35

#define enable_pin_1 21
#define enable_pin_2 16
#define input1_1 19
#define input1_2 18
#define input2_1 5
#define input2_2 17

#define default_speed 200
#define min_speed 100
#define max_speed 256

// Wi-Fi credentials
#define WIFI_SSID "Araby"       
#define WIFI_PASSWORD "AhmedAraby" 
#define AGENT_IP "192.168.107.223"           
#define AGENT_PORT 8888

// #define SERVO_VCC 17    
#define SERVO_PIN 23
#define MAGNET_PIN 32
#define UPPER_LIMIT 180
#define LOWER_LIMIT 0

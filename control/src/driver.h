#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>
#include "Config.h"

// driverPins[0] = IN1
// driverPins[1] = IN2
// driverPins[2] = IN3
// driverPins[3] = IN4
// driverPins[4] = ENA
// driverPins[5] = ENB

/* Note: Invalid pins in esp32 = {
    0, 1, 3, 6, 7, 8, 9, 10, 11, 12, 15,
}*/

class MotorDriver
{
private:
    const uint8_t driverPins[6] = {input1_1, input1_2, input2_1, input2_2, enable_pin_1, enable_pin_2};
    int stateMotorA; // 0: static, 1: forward, -1: backward
    int stateMotorB; // 0: static, 1: forward, -1: backward
    int speed = default_speed;
    // PWM channels for ESP32
    int ledc_channel_1 = 0;
    int ledc_channel_2 = 1;

public:
    void moveForward();
    void moveBackward();
    void rotateRight();
    void rotateLeft();
    void stop();
    void initialMotors();
    void setSpeed(int newSpeed);
    int getCurrentSpeed();
    void increaseSpeed();
    void decreaseSpeed();
};

#endif
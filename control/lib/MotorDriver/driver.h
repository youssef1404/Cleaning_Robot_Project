#ifndef DRIVER_H
#define DRIVER_H

#include "config.h"
#include <Arduino.h>


// driverPins[0] = IN1 
// driverPins[1] = IN2 
// driverPins[2] = IN3 
// driverPins[3] = IN4 
// driverPins[4] = ENA 
// driverPins[5] = ENB 

/* Note: Invalid pins in esp32 = {
    0, 1, 3, 6, 7, 8, 9, 10, 11, 12, 15, 
}*/

#define MOTOR_IN1 12
#define MOTOR_IN2 13
#define MOTOR_IN3 14
#define MOTOR_IN4 15
#define MOTOR_ENA 16
#define MOTOR_ENB 17

#define MAX_SPEED 255
#define MIN_SPEED 0
#define DEFAULT_SPEED 50

class MotorDriver
{
    private:
        const uint8_t driverPins[6] = {MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4, MOTOR_ENA, MOTOR_ENB};
        int stateMotorA;     // 0: static, 1: forward, -1: backward
        int stateMotorB;     // 0: static, 1: forward, -1: backward
        int speed;

    public:
        void moveForward();
        void moveBackward();
        void rotateRight();
        void rotateLeft();
        void stop();
        void initialMotors();
        void setSpeed(int newSpeed);
        int getCurrentSpeed();
};

#endif

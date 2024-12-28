#include "driver.h"
#include <Arduino.h>
#include "config.h"

void MotorDriver::initialMotors(){
    // Configure pins as outputs
    for (int i = 0; i < 6; i++){
        pinMode(this->driverPins[i], OUTPUT);
    }
    analogWrite(enable_pin_1, default_speed);
	analogWrite(enable_pin_2, default_speed);
    // stop the motors
    MotorDriver::stop();
}

void MotorDriver::moveForward() {
    Serial.println("moving Forward");
    // Motor A forward
    digitalWrite(this->driverPins[0], LOW); // IN1
    digitalWrite(this->driverPins[1], HIGH); // IN2
    // Motor B forward
    digitalWrite(this->driverPins[2], LOW); // IN3
    digitalWrite(this->driverPins[3], HIGH); // IN4

    this->stateMotorA = 1;
    this->stateMotorB = 1;
}

void MotorDriver::moveBackward() {
    Serial.println("moving Backward");
    // Motor A backward
    digitalWrite(this->driverPins[0], HIGH); // IN1
    digitalWrite(this->driverPins[1], LOW); // IN2
    
    // Motor B forward
    digitalWrite(this->driverPins[2], HIGH); // IN3
    digitalWrite(this->driverPins[3], LOW); // IN4
}

void MotorDriver::rotateRight() {
    Serial.println("moving Right");
    // Rotate in place by stopping one motor
    digitalWrite(this->driverPins[0], LOW); // IN1
    digitalWrite(this->driverPins[1], HIGH); // IN2
    digitalWrite(this->driverPins[2], LOW); // IN3
    digitalWrite(this->driverPins[3], LOW); // IN4
}

void MotorDriver::rotateLeft() {
    Serial.println("moving Left");
    // Rotate in place by stopping one motor
    digitalWrite(this->driverPins[0], LOW); // IN1
    digitalWrite(this->driverPins[1], LOW); // IN2
    digitalWrite(this->driverPins[2], LOW); // IN3
    digitalWrite(this->driverPins[3], HIGH); // IN4
}

void MotorDriver::stop() {
    for(int i = 0; i < 4; i++){
        digitalWrite(this->driverPins[i], LOW);
    }
}

int MotorDriver::getCurrentSpeed(){
    return this->speed;
}

void MotorDriver::setSpeed(int newSpeed){
    this->speed = newSpeed;
}

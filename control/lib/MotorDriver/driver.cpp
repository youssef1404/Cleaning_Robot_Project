#include "driver.h"
#include <Arduino.h>
#include "config.h"

void MotorDriver::initialMotors(){
    // Configure pins as outputs
    for (int i = 0; i < 6; i++){
        pinMode(this->driverPins[i], OUTPUT);
    }
    // stop the motors
    MotorDriver::stop();
}

void MotorDriver::moveForward() {
    // Motor A forward
    digitalWrite(this->driverPins[0], LOW); // IN1
    digitalWrite(this->driverPins[1], HIGH); // IN2
    // Motor B forward
    digitalWrite(this->driverPins[2], LOW); // IN3
    digitalWrite(this->driverPins[3], HIGH); // IN4

    this->stateMotorA = 1;
    this->stateMotorB = 1;
    
    // Set speed for both motors
    analogWrite(this->driverPins[4], DEFAULT_SPEED); // ENA
    analogWrite(this->driverPins[5], DEFAULT_SPEED); // ENB
}

void MotorDriver::moveBackward() {
    // Motor A backward
    digitalWrite(this->driverPins[0], HIGH); // IN1
    digitalWrite(this->driverPins[1], LOW); // IN2
    
    // Motor B forward
    digitalWrite(this->driverPins[2], HIGH); // IN3
    digitalWrite(this->driverPins[3], LOW); // IN4
    
    // Set speed for both motors
    analogWrite(this->driverPins[4], DEFAULT_SPEED); // ENA
    analogWrite(this->driverPins[5], DEFAULT_SPEED); // ENB
}

void MotorDriver::rotateRight() {
    // Rotate in place by stopping one motor
    digitalWrite(this->driverPins[0], LOW); // IN1
    digitalWrite(this->driverPins[1], LOW); // IN2
    digitalWrite(this->driverPins[2], LOW); // IN3
    digitalWrite(this->driverPins[3], HIGH); // IN4
    
    // Set speed for both motors
    analogWrite(this->driverPins[4], 0); // ENA
    analogWrite(this->driverPins[5], DEFAULT_SPEED); // ENB
}

void MotorDriver::rotateLeft() {
    // Rotate in place by stopping one motor
    digitalWrite(this->driverPins[0], LOW); // IN1
    digitalWrite(this->driverPins[1], HIGH); // IN2
    digitalWrite(this->driverPins[2], LOW); // IN3
    digitalWrite(this->driverPins[3], LOW); // IN4
    
    // Set speed for both motors
    analogWrite(this->driverPins[4], DEFAULT_SPEED); // ENA
    analogWrite(this->driverPins[5], 0); // ENB
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

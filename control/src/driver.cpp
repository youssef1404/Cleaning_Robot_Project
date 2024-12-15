#include "driver.h"
#include <Arduino.h>
#include "config.h"

void MotorDriver::configureMotorsPins(){
    pinMode(this->IN1, OUTPUT);
    pinMode(this->IN2, OUTPUT);
    pinMode(this->IN3, OUTPUT);
    pinMode(this->IN4, OUTPUT);
    pinMode(this->ENA, OUTPUT);
    pinMode(this->ENB, OUTPUT);
}

void MotorDriver::initialState(){
    digitalWrite(this->IN1, LOW);
	digitalWrite(this->IN2, LOW);
	digitalWrite(this->IN3, LOW);
	digitalWrite(this->IN4, LOW);
}

MotorDriver::MotorDriver(unsigned int in1, unsigned int in2, unsigned int in3,
                    unsigned int in4, unsigned int enA, unsigned int enB, int speed = DEFAULT_SPEED)
{
    if (!isValidPin(in1) || !isValidPin(in2) || 
        !isValidPin(in3) || !isValidPin(in4) || 
        !isValidPin(enA) || !isValidPin(enB)) {
        _motorError = true;
        _lastErrorTime = millis();
        return;
    }
    this->IN1 = in1;
    this->IN2 = in2;
    this->IN3 = in3;
    this->IN4 = in4;
    this->ENA = enA;
    this->ENB = enB;
    this->speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Initialize error states
    _motorError = false;
    _lastErrorTime = 0;

    // Configure and initialize
    configureMotorsPins();
    initialState();
}

// Pin validation specific to ESP32
bool MotorDriver::isValidPin(unsigned int pin) {
    // ESP32 specific pin validation
    // Exclude special pins and define valid GPIO ranges
    const int invalidPins[] = {
        // Add pins that shouldn't be used (like strapping pins)
        0, 1, 3, 6, 7, 8, 9, 10, 11, 12, 15, 
    };

    // Check against invalid pins
    for (int invalidPin : invalidPins) {
        if (pin == invalidPin) return false;
    }

    // General ESP32 GPIO range check
    return (pin >= 2 && pin <= 39);
}

// Error handling method
void MotorDriver::handleError() {
    // Implement error handling strategy
    if (millis() - this->_lastErrorTime > 5000) {  // 5-second error cool-down
        // Attempt to reset
        initialState();
        _motorError = false;
        _lastErrorTime = millis();
    }
}

void MotorDriver::forward() {
    if (_motorError) {
            handleError();
            return;
        }

    // Motor A forward
    digitalWrite(this->IN1, LOW);
    digitalWrite(this->IN2, HIGH);
    
    // Motor B forward
    digitalWrite(this->IN3, LOW);
    digitalWrite(this->IN4, HIGH);
    
    // Set speed for both motors
    analogWrite(this->ENA, this->speed);
    analogWrite(this->ENB, this->speed);
}

void MotorDriver::backward() {
    if (_motorError) {
        handleError();
        return;
    }
    // Motor A backward
    digitalWrite(this->IN1, HIGH);
    digitalWrite(this->IN2, LOW);
    
    // Motor B backward
    digitalWrite(this->IN3, HIGH);
    digitalWrite(this->IN4, LOW);
    
    // Set speed for both motors
    analogWrite(this->ENA, this->speed);
    analogWrite(this->ENB, this->speed);
}

void MotorDriver::rotateRight() {
    if (_motorError) {
        handleError();
        return;
    }
    // Rotate in place by stopping one motor
    digitalWrite(this->IN1, LOW);
    digitalWrite(this->IN2, LOW);
    digitalWrite(this->IN3, LOW);
    digitalWrite(this->IN4, HIGH);
    
    analogWrite(this->ENA, 0);
    analogWrite(this->ENB, this->speed);
}

void MotorDriver::rotateLeft() {
    if (_motorError) {
            handleError();
            return;
        }
    // Rotate in place by stopping one motor
    digitalWrite(this->IN1, LOW);
    digitalWrite(this->IN2, HIGH);
    digitalWrite(this->IN3, LOW);
    digitalWrite(this->IN4, LOW);
    
    analogWrite(this->ENA, this->speed);
    analogWrite(this->ENB, 0);
}

void MotorDriver::stop() {
    if (_motorError) {
            handleError();
            return;
        }
    // Stop both motors
    digitalWrite(this->IN1, LOW);
    digitalWrite(this->IN2, LOW);
    digitalWrite(this->IN3, LOW);
    digitalWrite(this->IN4, LOW);
    
    analogWrite(this->ENA, 0);
    analogWrite(this->ENB, 0);
}


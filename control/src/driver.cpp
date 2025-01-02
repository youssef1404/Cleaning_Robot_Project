#include "driver.h"
#include <Arduino.h>
#include "config.h"

  

    // MotorDriver() : speed(default_speed), stateMotorA(0), stateMotorB(0) {}

    // Initialize motor pins and PWM
    void MotorDriver::initialMotors() {
        // Configure pins as outputs for motor control
        pinMode(input1_1, OUTPUT);
        pinMode(input1_2, OUTPUT);
        pinMode(input2_1, OUTPUT);
        pinMode(input2_2, OUTPUT);
        pinMode(enable_pin_1, OUTPUT);
        pinMode(enable_pin_2, OUTPUT);

        // Configure PWM for motor speed control
        ledcSetup(this->ledc_channel_1, 5000, 8); // Frequency = 5kHz, Resolution = 8 bits
        ledcSetup(this->ledc_channel_2, 5000, 8);

        ledcAttachPin(enable_pin_1, this->ledc_channel_1);  // Attach pins to PWM channels
        ledcAttachPin(enable_pin_2, this->ledc_channel_2);

        // Set initial motor speed
        ledcWrite(this->ledc_channel_1, default_speed);
        ledcWrite(this->ledc_channel_2, default_speed);

        // Stop the motors initially
        MotorDriver::stop();
    }

    // Move forward
    void MotorDriver::moveForward() {
        Serial.println("Moving Forward");

        // Motor A forward
        digitalWrite(input1_1, LOW);  // IN1
        digitalWrite(input1_2, HIGH); // IN2

        // Motor B forward
        digitalWrite(input2_1, LOW);  // IN3
        digitalWrite(input2_2, HIGH); // IN4

        this->stateMotorA = 1;
        this->stateMotorB = 1;
    }

    // Move backward
    void MotorDriver::moveBackward() {
        Serial.println("Moving Backward");

        // Motor A backward
        digitalWrite(input1_1, HIGH); // IN1
        digitalWrite(input1_2, LOW);  // IN2

        // Motor B backward
        digitalWrite(input2_1, HIGH); // IN3
        digitalWrite(input2_2, LOW);  // IN4

        this->stateMotorA = 0;
        this->stateMotorB = 0;
    }

    // Rotate right (rotate in place)
    void MotorDriver::rotateRight() {
        Serial.println("Rotating Right");

        // Stop one motor to rotate in place
        digitalWrite(input1_1, LOW);  // IN1
        digitalWrite(input1_2, HIGH); // IN2
        digitalWrite(input2_1, LOW);  // IN3
        digitalWrite(input2_2, LOW);  // IN4
    }

    // Rotate left (rotate in place)
    void MotorDriver::rotateLeft() {
        Serial.println("Rotating Left");

        // Stop one motor to rotate in place
        digitalWrite(input1_1, LOW);  // IN1
        digitalWrite(input1_2, LOW);  // IN2
        digitalWrite(input2_1, LOW);  // IN3
        digitalWrite(input2_2, HIGH); // IN4
    }

    // Stop the motors
    void MotorDriver::stop() {
        digitalWrite(input1_1, LOW);
        digitalWrite(input1_2, LOW);
        digitalWrite(input2_1, LOW);
        digitalWrite(input2_2, LOW);
    }

    // Get the current speed
    int MotorDriver::getCurrentSpeed() {
        return this->speed;
    }

    // Set a new speed
    void MotorDriver::setSpeed(int newSpeed) {
        Serial.print("Speed: ");
        Serial.println(newSpeed);

        // Ensure the new speed is within the limits
        if (newSpeed >= min_speed && newSpeed <= max_speed) {
            this->speed = newSpeed;

            // Apply PWM to adjust speed
            ledcWrite(this->ledc_channel_1, this->speed);  // Motor 1 speed
            ledcWrite(this->ledc_channel_2, this->speed);  // Motor 2 speed
        }
    }

    // Increase the speed
    void MotorDriver::increaseSpeed() {
        int current = this->getCurrentSpeed();
        Serial.print("Increasing, current speed: ");
        Serial.println(current);

        if (current < max_speed) {
            this->setSpeed(current + 5);  // Increase speed by 5
        }
    }

    // Decrease the speed
    void MotorDriver::decreaseSpeed() {
        int current = getCurrentSpeed();
        Serial.print("Decreasing, current speed: ");
        Serial.println(current);

        if (current > min_speed) {
            this->setSpeed(current - 5);  // Decrease speed by 5
        }
    }

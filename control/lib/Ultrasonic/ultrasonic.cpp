// ultrasonic.cpp
#include "ultrasonic.h"

UltrasonicSensor::UltrasonicSensor(uint8_t trig, uint8_t echo, unsigned long timeoutMicros)
{
    this->trigPin = trig;
    this->echoPin = echo;
    this->timeout = timeoutMicros;
    this->lastDistance = 0;
}

void UltrasonicSensor::init()
{
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
}

float UltrasonicSensor::getDistance()
{
    // Clear trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Send 10μs pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read echo pin, return pulse duration in microseconds
    unsigned long duration = pulseIn(echoPin, HIGH, timeout);

    // Calculate distance: speed of sound (343m/s) / 2 (round trip)
    // (duration * 0.0343) / 2
    this->lastDistance = duration * 0.01715; // Returns distance in centimeters

    return this->lastDistance;
}

float UltrasonicSensor::getDistanceInch()
{
    // Convert centimeters to inches
    return getDistance() * 0.393701;
}

bool UltrasonicSensor::isObstacleDetected(float threshold)
{
    float currentDistance = getDistance();
    return currentDistance <= threshold && currentDistance > 0;
}

float UltrasonicSensor::getLastReading()
{
    return this->lastDistance;
}

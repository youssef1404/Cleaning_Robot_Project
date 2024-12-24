// ultrasonic.h
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class UltrasonicSensor
{
private:
    uint8_t trigPin;
    uint8_t echoPin;
    unsigned long timeout;
    float lastDistance;

public:
    // Constructor
    UltrasonicSensor(uint8_t trig, uint8_t echo, unsigned long timeoutMicros = 25000);

    // Core functions
    void init();
    float getDistance();     // Returns distance in centimeters
    float getDistanceInch(); // Returns distance in inches

    // Utility functions
    bool isObstacleDetected(float threshold);
    float getLastReading();
};

#endif

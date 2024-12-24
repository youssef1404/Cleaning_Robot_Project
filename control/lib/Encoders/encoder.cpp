#include "encoder.h"
#include <Arduino.h>

Encoder::Encoder(int pinA, int pinB, float pulsesPerRev)
{
    encoderPinA = pinA;
    encoderPinB = pinB;
    ppr = pulsesPerRev;
    count = 0;
    lastTime = 0;
    rpm = 0;
}

void Encoder::init()
{
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    lastTime = millis();
}

void Encoder::updateCount()
{
    // Called by interrupt
    // Read second pin to determine direction
    if (digitalRead(encoderPinB) == HIGH)
    {
        count++;
    }
    else
    {
        count--;
    }

    // Calculate RPM every 100ms
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 100)
    {
        // RPM = (pulses / pulses_per_rev) * (60s / time_in_minutes)
        rpm = (float)(count * 600.0) / (ppr * (currentTime - lastTime));
        lastTime = currentTime;
        count = 0; // Reset counter
    }
}

long Encoder::getCount()
{
    return count;
}

float Encoder::getRPM()
{
    return rpm;
}

void Encoder::reset()
{
    count = 0;
    rpm = 0;
}

float Encoder::getDistance(float wheelDiameter)
{
    // Distance = wheel circumference * number of rotations
    // number of rotations = count / ppr
    return (PI * wheelDiameter * count) / ppr;
}
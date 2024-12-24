#ifndef ENCODER_H
#define ENCODER_H

class Encoder
{
private:
    int encoderPinA;        // First encoder pin
    int encoderPinB;        // Second encoder pin
    volatile long count;    // Pulse count
    float ppr;              // Pulses per revolution
    unsigned long lastTime; // Last time reading
    float rpm;              // Current RPM

public:
    Encoder(int pinA, int pinB, float pulsesPerRev);
    void init();
    void updateCount();
    long getCount();
    float getRPM();
    void reset();
    float getDistance(float wheelDiameter); // Distance in cm
};

#endif
#ifndef ENCODER_H
#define ENCODER_H
#include"Arduino.h"

class Encoder {
public:
long encoder_counts;

Encoder(int resoultion);

float calculate_speed();

private:
long prev_counts;
float prev_time;
int resoultion;
};

#endif
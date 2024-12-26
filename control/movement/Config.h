#ifndef PINCONFIG_H
#define PINCONFIG_H

#define DEADZONE 4
#define OUTPUTLIMITS 65000.0
#define RESOLUTION 200.0
#define TIME_FREQ 30
#define dt 0.1

#define kp0 1
#define kp1 0

#define ki0 1
#define ki1 0

#define kd0 1
#define kd1 0

#define pin_A1  PB6 // 36
#define pin_A2 PB7 //34

#define pin_B1 PA3 // 39
#define pin_B2 PA2   //35

#define enable_pin_1 PB0   //23
#define enable_pin_2 PA7    //22
#define input1_1  PA6     //21
#define input1_2  PB7    //19
#define input2_1  PB13   //18
#define input2_2  PB12 //5

#endif
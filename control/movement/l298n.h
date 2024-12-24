#ifndef L298N_H
#define L298N_H

#include <Arduino.h>


class L298N {

private:
    int speed;
    int direction;
    int enable_pin, input1, input2;


public:
    L298N(int enable_pin,int input1, int input2);
    void driver_init();
    void set_speed(int speed);
    void set_direction(int direction);
    void control_speed();
};

#endif
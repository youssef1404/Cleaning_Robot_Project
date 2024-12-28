#ifndef _ROVER_H_
#define _ROVER_H_

#include "RosComm.h"
#include "mechServo.h"
#include "driver.h"
#include "ultrasonic.h"
#include "encoder.h"


class Rover
{
    private:
        MechServo servo;
        RosComm my_comm;
        MotorDriver driver;
        void updateDistance();
        void move();
        int keyValue;
    public:
        void setup();
        void loop();
};


#endif
#ifndef _MECHSERVO_H_
#define _MECHSERVO_H_

#include <ESP32Servo.h>
#include "Config.h"


class MechServo
{
    private:
        Servo mechServo;
        int currentAngle;

    public:
        void init();
        void move(int button);
        int getAngle();
        void tiltUp();
        void tiltDown();
};

#endif
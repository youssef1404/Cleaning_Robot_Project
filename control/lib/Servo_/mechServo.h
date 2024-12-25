#ifndef _MECHSERVO_H_
#define _MECHSERVO_H_

#include <ESP32Servo.h>

#define SERVO_PIN 15
#define UPPER_LIMIT 180
#define LOWER_LIMIT 0
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
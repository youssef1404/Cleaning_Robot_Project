#ifndef DRIVER_H
#define DRIVER_H

class MotorDriver
{
    protected:
        unsigned int IN1;
        unsigned int IN2;
        unsigned int IN3;
        unsigned int IN4;
        unsigned int ENA;
        unsigned int ENB;
        int stateA;     // 0: static, 1: forward, -1: backward
        int stateB;     // 0: static, 1: forward, -1: backward
        int speed;
    private:
        // Add error tracking
        bool _motorError;
        unsigned long _lastErrorTime;
    public:
        MotorDriver (unsigned int in1, unsigned int in2, unsigned int in3,
                    unsigned int in4, unsigned int enA, unsigned int enB, int speed);
        void forward();
        void backward();
        void rotateRight();
        void rotateLeft();
        void stop();
        void initialState();
        void configureMotorsPins();
        void setSpeed(int newSpeed);
        int getCurrentSpeed() const;
        bool isValidPin(unsigned int pin);
        void handleError();
};


#endif
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>
#include "Config.h"
#include <cmath>

class PIDController {
  private:
    float kp;
    float ki;
    float kd;
    float error, integral, derivative;
    float setpoint;
    float deadzone;
    float sample_time;
    float last_error;
    float output;

  public:
    PIDController(float Kp, float Ki, float Kd,float deadzone,float sample_time);
    float calculateOutput(float measurement);
    void setSetpoint(float newSetpoint);
    void setParameters(float Kp, float Ki, float Kd);

};

#endif

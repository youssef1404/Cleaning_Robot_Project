#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include "Config.h"
#include <cmath>

class PIDController {
  private:
    float kp, ki, kd;
    float error, integral, lastError;
    float setpoint;
    float outputLimits;
    float deadzone;

  public:
    PIDController(double Kp, double Ki, double Kd, float outputLimits,float deadzone);
    float calculateOutput(float measurement);
    void setSetpoint(float newSetpoint);
    void setParameters(double Kp, double Ki, double Kd);

    float getSetpoint() const;

};

#endif

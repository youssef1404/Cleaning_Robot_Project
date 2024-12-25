#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include "Config.h"
#include <cmath>

class PIDController {
public:
  PIDController(double Kp, double Ki, double Kd, float outputLimits,float deadzone);
  float calculateOutput(float measurement);
  void setSetpoint(float newSetpoint);
  void setParameters(double Kp, double Ki, double Kd);

  float getSetpoint() const;

private:
  float _Kp, _Ki, _Kd;
  float _error, _integral, _lastError;
  float _setpoint;
  float _outputLimits;
  float _deadzone;
};

#endif

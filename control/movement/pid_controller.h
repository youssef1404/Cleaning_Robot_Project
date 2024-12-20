#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <cmath>

class PIDController {
public:
  PIDController(double Kp, double Ki, double Kd, float outputLimits,float deadzone);
  float calculateOutput(float measurement, float dt);
  void setSetpoint(float newSetpoint);
  float getSetpoint() const;

private:
  float _Kp, _Ki, _Kd;
  float _error, _integral, _lastError;
  float _setpoint;
  float _outputLimits;
  float _deadzone;
};

#endif

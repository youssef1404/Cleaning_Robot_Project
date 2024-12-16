#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
  PIDController(float Kp, float Ki, float Kd);
  float calculateOutput(float setpoint, float measurement, float dt);

private:
  float _Kp, _Ki, _Kd;
  float _error, _integral, _lastError;
};

#endif

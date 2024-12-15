#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
  PIDController(float Kp, float Ki, float Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _error = 0;
    _integral = 0;
    _lastError = 0;
  }

  float calculateOutput(float setpoint, float measurement) {
    _error = setpoint - measurement;
// Restrict the error
  float maxError = 0; // to max error value
    _error = constrain(_error, -maxError, maxError);
    _integral += _error * dt;
  float derivative = (_error - _lastError) / dt;
    _lastError = _error;

    float output = _Kp * _error + _Ki * _integral + _Kd * derivative;
    return output;
  }

private:
  float _Kp, _Ki, _Kd;
  float _error, _integral, _lastError;
};

#endif

#include "pid_controller.h"
PIDController::PIDController(float Kp, float Ki, float Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _error = 0;
  _integral = 0;
  _lastError = 0;
}

float PIDController::calculateOutput(float setpoint, float measurement,float dt) {
  _error = setpoint - measurement;
  _integral += _error * dt;
  float derivative = (_error - _lastError) / dt;
  _lastError = _error;

  float output = _Kp * _error + _Ki * _integral + _Kd * derivative;
  return output;
}

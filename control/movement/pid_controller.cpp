#include "pid_controller.h"

PIDController::PIDController(double Kp, double Ki, double Kd, float outputLimits,float deadzone)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd),
      _error(0.0), _integral(0.0), _lastError(0.0),
      _setpoint(0.0), _outputLimits(outputLimits),
      _deadzone(deadzone){}

float PIDController::calculateOutput(float measurement) {
  _error = _setpoint - measurement;

  // DeadZone : Limit the error term
  if (abs(_error) < _deadzone) { _error = 0.0; }

  // Anti-windup: Limit the integral term
  if (abs(_error) > _outputLimits) {
      _integral -= _error * dt;
  } else {
      _integral += _error * dt;
  }

  float derivative = (_error - _lastError) / dt;
  _lastError = _error;

  float output = _Kp * _error + _Ki * _integral + _Kd * derivative;

  // Saturate the output
  if (output > _outputLimits) {
      output = _outputLimits;
  } else if (output < -_outputLimits) {
      output = -_outputLimits;
  }

  return output;
}

void PIDController::setParameters(double Kp, double Ki, double Kd):_Kp(Kp), _Ki(Ki), _Kd(Kd) {

}
void PIDController::setSetpoint(float newSetpoint) {
  _setpoint = newSetpoint;
}

float PIDController::getSetpoint() const {
  return _setpoint;
}

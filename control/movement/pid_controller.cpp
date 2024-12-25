#include "pid_controller.h"

PIDController::PIDController(double kp, double ki, double kd, float outputLimits,float deadzone)
    : kp(kp), ki(ki), kd(kd),
      error(0.0), integral(0.0), lastError(0.0),
      setpoint(0.0), outputLimits(outputLimits),
      deadzone(deadzone){}

float PIDController::calculateOutput(float measurement) {
  this->error = this->setpoint - measurement;

  // DeadZone : Limit the error term
  if (abs(this->error) < this->deadzone) { this->error = 0.0; }

  // Anti-windup: Limit the integral term
  if (abs(this->error) > this->outputLimits) {
      this->integral -= this->error * dt;
  } else {
      this->integral += this->error * dt;
  }

  float derivative = (this->error - this->lastError) / dt;
  this->lastError = this->error;

  float output = this->kp * this->error + this->ki * this->integral + this->kd * derivative;

  // Saturate the output
  if (output > this->outputLimits) {
      output = this->outputLimits;
  } else if (output < -this->outputLimits) {
      output = -this->outputLimits;
  }

  return output;
}

void PIDController::setParameters(double kp, double ki, double kd){
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PIDController::setSetpoint(float newSetpoint) {
  this->setpoint = newSetpoint;
}

float PIDController::getSetpoint() const {
  return this->setpoint;
}

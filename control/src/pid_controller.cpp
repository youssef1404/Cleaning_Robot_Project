#include "pid_controller.h"

// Constructor
PIDController::PIDController(float Kp, float Ki, float Kd, float outputLimits, float deadzone, float dt, float alpha)
    : kp(Kp), ki(Ki), kd(Kd), outputLimits(outputLimits), deadzone(deadzone), dt(dt), alpha(alpha),
      error(0.0), integral(0.0), lastError(0.0), derivativeFiltered(0.0) {}

// Calculate PID output
float PIDController::calculateOutput(float measurement)
{
  // Calculate error
  error = setpoint - measurement;

  // Apply deadzone
  if (std::abs(error) < deadzone)
  {
    error = 0.0;
  }

  // Proportional term
  float Pout = kp * error;

  // Integral term with anti-windup
  integral += error * dt;
  if (std::abs(integral) > outputLimits)
  {
    integral = std::copysign(outputLimits, integral);
  }
  float Iout = ki * integral;

  // Derivative term with filtering
  float derivative = (error - lastError) / dt;
  derivativeFiltered = alpha * derivativeFiltered + (1 - alpha) * derivative;
  float Dout = kd * derivativeFiltered;

  // Calculate total output and apply limits
  float output = Pout + Iout + Dout;
  if (std::abs(output) > outputLimits)
  {
    output = std::copysign(outputLimits, output);
  }

  // Update last error
  lastError = error;

  return output;
}

// Setters
void PIDController::setSetpoint(float newSetpoint) { setpoint = newSetpoint; }
void PIDController::setParameters(float Kp, float Ki, float Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}
void PIDController::setDt(float newDt) { dt = newDt; }
void PIDController::setOutputLimits(float limit) { outputLimits = limit; }

// Getters
float PIDController::getSetpoint() const { return setpoint; }
float PIDController::getKp() const { return kp; }
float PIDController::getKi() const { return ki; }
float PIDController::getKd() const { return kd; }
float PIDController::getDt() const { return dt; }
float PIDController::getOutputLimits() const { return outputLimits; }

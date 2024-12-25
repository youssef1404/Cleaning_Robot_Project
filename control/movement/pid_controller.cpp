#include "pid_controller.h"


PIDController::PIDController(float kp, float ki, float kd,float deadzone, float sample_time)
    : kp(kp), ki(ki), kd(kd),
      error(0.0), integral(0.0), derivative(0),
      setpoint(0.0),
      deadzone(deadzone),sample_time(sample_time),last_error(0),output(0){}




float PIDController::calculateOutput(float measurement) {


  error = setpoint - measurement;

  integral += ki*(error * sample_time);
  integral = constrain(integral, integral_limit, -integral_limit);
  derivative = kd * (error - last_error)/sample_time;
  output = kp * error + integral + derivative;
  last_error = error;
  if((abs(setpoint - measurement) < deadzone) && (0 == setpoint)){
    output = 0;
    integral = 0;
  }
  return output;
}


void PIDController::setParameters(float kp, float ki, float kd){
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}



void PIDController::setSetpoint(float newSetpoint) {
  this->setpoint = newSetpoint;
}



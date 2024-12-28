#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include "Config.h"
#include <cmath>

class PIDController
{
private:
  float kp, ki, kd;                 // PID gains
  float error, integral, lastError; // Error terms
  float setpoint;                   // Desired setpoint
  float outputLimits;               // Output limits
  float deadzone;                   // Deadzone range
  float dt;                         // Time step
  float derivativeFiltered;         // Filtered derivative
  float alpha;                      // Filter coefficient

public:
  // Constructor with additional parameters for dt and filter coefficient
  PIDController(float Kp, float Ki, float Kd, float outputLimits, float deadzone, float dt, float alpha);

  // Compute PID output
  float calculateOutput(float measurement);

  // Setters
  void setSetpoint(float newSetpoint);
  void setParameters(float Kp, float Ki, float Kd);
  void setDt(float newDt); // Update dt if needed dynamically

  // Getters
  float getSetpoint() const;
  float getKp() const; // Get proportional gain
  float getKi() const; // Get integral gain
  float getKd() const; // Get derivative gain
  float getDt() const; // Get time step

  // Output limits management
  void setOutputLimits(float limit); // Set output limits dynamically
  float getOutputLimits() const;     // Get output limits
};

#endif

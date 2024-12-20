#include <iostream>
#include <thread>
#include <chrono>
#include "pid_controller.h"

int main() {
    PIDController pidController(0.5, 0.1, 0.2,150.0,5.0);

    float setpoint;
    std::cout << "Enter the desired setpoint: ";
    std::cin >> setpoint;

    pidController.setSetpoint(setpoint);

    float processVariable = 0.0;
    float dt = 0.1; // Time step

    while (true) {
        float error = setpoint - processVariable;
        float output = pidController.calculateOutput(processVariable, dt);

        // Simulate the process (replace with your actual system)
        processVariable += output * dt;

        std::cout << "Error: " << error << ", Output: " << output << ", Process Variable: " << processVariable << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    return 0;
}

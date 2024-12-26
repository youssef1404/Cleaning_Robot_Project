#include "rover.h"

void Rover::setup(){
    Serial.begin(115200);
    Serial.println("Hello Bor3y");

    // initialize communication 
    my_comm.initialize();

    // Initialize motors
    driver.initialMotors();

    // Initialize servo
    servo.init();
}

void Rover::loop(){
    my_comm.loop();
    this->keyValue = my_comm.getkeyboardValue();
    Serial.println("keyboard value: ");
    Serial.println(this->keyValue);
    this->move();
}

void Rover::move(){
    if (this->keyValue == 1) driver.moveForward();
    if (this->keyValue == 2) driver.moveBackward();
    if (this->keyValue == 3) driver.rotateRight();
    if (this->keyValue == 4) driver.rotateLeft();
    if (this->keyValue == 5) servo.move(100);
    if (this->keyValue == 6) servo.move(0);
}

#include "mechServo.h"
#include "config.h"

void MechServo::init(){
    this->mechServo.attach(SERVO_PIN);
    this->currentAngle = LOWER_LIMIT;
    pinMode(SERVO_VCC, OUTPUT);
    pinMode(MAGNET_PIN, OUTPUT);
    digitalWrite(MAGNET_PIN, LOW);
    digitalWrite(SERVO_VCC, HIGH);
}

void MechServo::tiltDown(){
    this->currentAngle -= 1;
    if(this->currentAngle < LOWER_LIMIT) this->currentAngle = LOWER_LIMIT;
    this->mechServo.write(this->currentAngle); 
}

void MechServo::tiltUp(){
    this->currentAngle += 1;
    if(this->currentAngle > UPPER_LIMIT) this->currentAngle = UPPER_LIMIT;
    this->mechServo.write(this->currentAngle); 
}

void MechServo::move(int but){
    this->currentAngle = but;
    if (this->currentAngle < LOWER_LIMIT) this->currentAngle = LOWER_LIMIT;
    if (this->currentAngle > UPPER_LIMIT) this->currentAngle = UPPER_LIMIT;
    this->mechServo.write(this->currentAngle);
}

int MechServo::getAngle(){
    return this->currentAngle;
}

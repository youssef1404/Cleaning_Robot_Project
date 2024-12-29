#include "mechServo.h"

void MechServo::init(){
    this->mechServo.attach(SERVO_PIN);
    this->currentAngle = LOWER_LIMIT;
    pinMode(SERVO_VCC, OUTPUT);
    pinMode(MAGNET_PIN, OUTPUT);
    digitalWrite(MAGNET_PIN, LOW);
    digitalWrite(SERVO_VCC, HIGH);
    this->mechServo.write(15);
    // delay(15);
}

void MechServo::tiltDown(){
    this->currentAngle -= 10;
    if(this->currentAngle < LOWER_LIMIT) this->currentAngle = LOWER_LIMIT;
    this->mechServo.write(this->currentAngle); 
}

void MechServo::tiltUp(){
    this->currentAngle += 10;
    if(this->currentAngle > UPPER_LIMIT) this->currentAngle = UPPER_LIMIT;
    this->mechServo.write(this->currentAngle); 
}

void MechServo::move(int but){
    // Serial.println("here1, vcc");
    // this->currentAngle = but;
    // if (this->currentAngle < LOWER_LIMIT) this->currentAngle = LOWER_LIMIT;
    // if (this->currentAngle > UPPER_LIMIT) this->currentAngle = UPPER_LIMIT;
    // this->mechServo.write(but);
    // Serial.println(this->currentAngle);
    // Serial.println("here2, vcc");
    if (this->currentAngle > but)
    {
        for (int i = currentAngle; i >= but; i--)
        {
            this->mechServo.write(i);
            delay(15);
        }
        
    } else {
        for(int i = currentAngle; i <= but; i++) {
            this->mechServo.write(i);
            delay(15);
        }
    }
    this->currentAngle = but;
    
}

int MechServo::getAngle(){
    return this->currentAngle;
}

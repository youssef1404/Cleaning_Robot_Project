#include "l298n.h"

L298N::L298N(int enable_pin, int input1, int input2):speed(0),direction(0){
    this-> enable_pin = enable_pin;
    this->input1 = input1;
    this->input2 = input2;
}
void driver_init(){
    pinMode(enable_pin, OUTPUT);
    pinMode(input1, OUTPUT);
    pinMode(input2, OUTPUT);

}
void set_speed(int speed){
    this->speed = abs(speed);
}

void set_direction(int speed){
    if (speed > 0)
        this->direction = 1; //forward
    else if (speed<0)
        this->direction = -1; //reverse
    else 
        this->direction = 0; //stop
}

void control_speed(){
    if (this->direction == 1) {
        digitalWrite(input1, HIGH);
        digitalWrite(input2, LOW);
        analogWrite(enable_pin, this->speed);
    } else if (this->direction == -1) {
        digitalWrite(input1, LOW);
        digitalWrite(input2, HIGH);
        analogWrite(enablePin, this->speed);
    } else {
        digitalWrite(input1, LOW);
        digitalWrite(input2, LOW);
        analogWrite(enablePin, 0);
    }
}
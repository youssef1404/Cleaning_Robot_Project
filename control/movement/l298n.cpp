#include <sys/_stdint.h>
#include "l298n.h"
#include <Arduino.h>

L298N::L298N(uint8_t enable_pin ,uint8_t input1, uint8_t input2):speed(0), direction(0) {
  this->enable_pin = enable_pin;
  this->input1_pin = input1;
  this->input2_pin = input2;
}

void L298N::driver_init(){
  pinMode(this->enable_pin, OUTPUT);
  pinMode(this->input1_pin, OUTPUT);
  pinMode(this->input2_pin, OUTPUT);
}
void L298N::set_speed(int speed){
  this->speed = abs(speed);
}

void L298N::set_direction(int speed){
  if (speed > 0)
    this->direction = 1; //forward
  else if (speed<0)
    this->direction = -1; //reverse
  else 
    this->direction = 0; //stop
}

void L298N::control_speed(){
  if (this->direction == 1) {
    digitalWrite(this->input1_pin, HIGH);
    digitalWrite(this->input2_pin, LOW);
    analogWrite(this->enable_pin, this->speed);
  } else if (this->direction == -1) {
    digitalWrite(this->input1_pin, LOW);
    digitalWrite(this->input2_pin, HIGH);
    analogWrite(this->enable_pin, this->speed);
  } else {
    digitalWrite(this->input1_pin, LOW);
    digitalWrite(this->input2_pin, LOW);
    analogWrite(this->enable_pin, 0);
  }
}
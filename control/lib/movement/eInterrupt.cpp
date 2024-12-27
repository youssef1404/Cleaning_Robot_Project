#include "eInterrupt.h"

eInterrupt::eInterrupt(int pin_A, int pin_B, float resoultion) {
  this->pin_A = pin_A;
  this->pin_B = pin_B;
  this->resoultion = resoultion;
}

float eInterrupt::calculate_speed(){
  float current_time=millis();
  long new_counts = encoder_counts;
  int delta_counts = new_counts - prev_counts;
  float DT = current_time - prev_time;
  float speed = (secondes_per_minute * milles/ resoultion)*(delta_counts/DT);
  prev_counts = new_counts;
  prev_time = current_time;
  return speed;
}

void eInterrupt::Init(){
  pinMode(pin_A, INPUT_PULLUP);
  pinMode(pin_B, INPUT_PULLUP);

}

void eInterrupt::ISR_A_routine(){
  if (digitalRead(pin_A) == digitalRead(pin_B)){
    encoder_counts--;
  }
  else{
    encoder_counts++;
  }
}

void eInterrupt::ISR_B_routine(){
  if (digitalRead(pin_A) != digitalRead(pin_B)){
    encoder_counts--; 
  }
  else{
    encoder_counts++;
  }
}
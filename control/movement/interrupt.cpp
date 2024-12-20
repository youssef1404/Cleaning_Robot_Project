#include "interrupt.h"

Interrupt::Interrupt(unit8_t pin_A, unit8_t pin_B, void(*ISR_A)(), void(*ISR_B)()):Encoder(resoultion)
    this->pin_A = pin_A;
    this->pin_B = pin_B;
    isrA(ISR_A),isrB(ISR_B){}


void Interrupt::Init(){
  pinMode(pin_A, INPUT_PULLUP);
  pinMode(pin_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pin_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_B), isrB, CHANGE);
}

void Interrupt::ISR_A_routine(){
  if (digitalRead(pin_A) == digitalRead(pin_B)){
    encoder_counts--;
  }
  else{
    encoder_counts++;
  }
}

void Interrupt::ISR_B_routine(){
  if (digitalRead(pin_A) != digitalRead(pin_B)){
    encoder_counts--; 
  }
  else{
    encoder_counts++;
  }
}
// #include <Arduino.h>
// #include "encoder.h"
// #include "config.h"

// int pos;

// Encoder::Encoder(unsigned int encA, unsigned int encB){
//     this->ENCA = encA;
//     this->ENCB = encB;
// }

// void IRAM_ATTR Encoder::encoderISR(void* arg){
//     int b = digitalRead(RIGHT_MOTOR_ENCA);
//     if (b > 0) pos++;
//     else pos--;
// }

// void Encoder::configureEncoder(){
//     pinMode(this->ENCA, INPUT); 
//     pinMode(this->ENCB, INPUT); 
//     attachInterruptArg(digitalPinToInterrupt(ENCA_Data), encoderISR, this, RISING);
// }

// void Encoder::setPosValue(int val){
//     this->pos = val;
// }

// int Encoder::getPosValue(){
//     return pos;
// }

#include <Timer.h>
#include "Config.h"
#include "eInterrupt.h"

Timer timer;

void update_encoder();
void Motor0_ISR_EncoderA();
void Motor0_ISR_EncoderB();
void Motor1_ISR_EncoderA();
void Motor1_ISR_EncoderB();



//creating Encoder objects
eInterrupt Interrupt[]{eInterrupt(pin_A1, pin_B1, RESOLUTION),
                      eInterrupt(pin_A2, pin_B2, RESOLUTION)};

void setup(){
  Serial.begin(115200);

  Interrupt[0].Init();
  Interrupt[1].Init();

    timer.every(TIME_FREQ, update_encoder);


    attachInterrupt(digitalPinToInterrupt(Interrupt[0].pin_A), Motor0_ISR_EncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Interrupt[0].pin_B), Motor0_ISR_EncoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Interrupt[1].pin_A), Motor1_ISR_EncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Interrupt[1].pin_B), Motor1_ISR_EncoderB, CHANGE);
}

void loop(){
      timer.update();
}

void update_encoder(){

    Serial.print("Encoder one counts");
    Serial.println(Interrupt[0].encoder_counts);
        Serial.print("Encoder two counts");
    Serial.println(Interrupt[1].encoder_counts);
        Serial.print("Encoder one speeds");
    Serial.println(Interrupt[0].calculate_speed());
        Serial.print("Encoder two speeds");
    Serial.println(Interrupt[1].calculate_speed());
}

void Motor0_ISR_EncoderA()
{
  Interrupt[0].ISR_A_routine();
}

void Motor0_ISR_EncoderB()
{
  Interrupt[0].ISR_B_routine();
}

void Motor1_ISR_EncoderA()
{
  Interrupt[1].ISR_A_routine();
}

void Motor1_ISR_EncoderB()
{
  Interrupt[1].ISR_B_routine();
}
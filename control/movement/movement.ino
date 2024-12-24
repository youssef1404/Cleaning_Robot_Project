#include "pid_controller.h"
#include "interrupt.h"
#include "l298n.h"
#include "Config.h"


void isrA1();
void isrB1();


void isrA2();
void isrB2();

//creating PID objects
PIDController PID[]{PIDController(kp0,ki0,kd0,outputLimits,deadzone),
                    PIDController(kp1,ki1,kd1,outputLimits,deadzone)};

//creating Encoder objects
Interrupt interrupt[]{Interrupt(pin_A1,pin_B1,isrA1,isrB1),
                      Interrupt(pin_A2,pin_B2,isrA2,isrB2)};

//creating motor driver objects
L298N l298n[]{
             L298N(enable_pin_1,input1_1,input2_1),
             L298N(enable_pin_2,input1_2,input2_2)
};

float encoder_feedback[] = {0,0};
float pid[] = {0,0};

void setup(){
    interrupt[0].Init();
    interrupt[1].Init();

    l298n[0].driver_init();
    l298n[1].driver_init();

    analogWriteResolution(16);

}
void loop(){


}

void update_encoder(){
    encoder_feedback[0] = interrupt[0].calculate_speed();
    encoder_feedback[1] = interrupt[0].calculate_speed();
}

void speed_controll(){
    pid[0]=PID[0].calculateOutput(encoder_feedback[0]);
    pid[1]=PID[1].calculateOutput(encoder_feedback[1]);

    l298n[0].set_speed(pid[0]);
    l298n[1].set_speed(pid[1]);
    l298n[0].set_direction(pid[0]);
    l298n[1].set_direction(pid[1]);

    l298n[0].control_speed();
    l298n[1].control_speed();
}

void isrA1() { Encoder1_Interrupt.ISR_A_routine();}
void isrB1() { Encoder1_Interrupt.ISR_B_routine();}


void isrA2() { interrupt[0].ISR_A_routine();}
void isrB2() { interrupt[1].ISR_B_routine();}
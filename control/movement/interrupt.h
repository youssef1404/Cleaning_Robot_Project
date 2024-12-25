#ifndef INTERRUPT_H
#define INTERRUPT_H
#include "Config.h"

//class Encoder;

class Interrupt{
        public:
            Interrupt(uint8_t pin_A, uint8_t pin_B,float resoultion);
            void Init();
            void ISR_A_routine();
            void ISR_B_routine();
            uint8_t pin_A;
            uint8_t pin_B;
            long encoder_counts = 0;
            float resoultion = 0; 
            float calculate_speed();
        private:
            long prev_counts= 0;
            float prev_time = 0;


};
#endif
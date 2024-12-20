#ifndef INTERRUPT_H
#define INTERRUPT_H
#include "encoder.h"


class Interrupt : public Encoder{
        public:
            Interrupt(unit8_t pin_A, unit8_t pin_B, void(*ISR_A)(), void(*ISR_B)());
            void Init();
            void ISR_A_routine();
            void ISR_B_routine():
        private:
            unit8_t pin_A;
            unit8_t pin_B;
            void(*isrA)();
            void(*isrB)();
};
#endif
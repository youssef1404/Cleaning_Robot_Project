#ifndef ENCODER_H_
#define ENCODER_H_

class Encoder
{
    private:
        unsigned int ENCA;
        unsigned int ENCB;
        int ENCA_Data;
        int ENCB_Data;
        int pos;
    public:
        Encoder (unsigned int encA, unsigned int encB);
        // void readEncoder();
        void configureEncoder();
        void setPosValue(int);
        int getPosValue();
        void encoderISR(void* arg);
};

#endif
#include "encoder.h"
Encoder::Encoder(int resoultion) {
    encoder_counts = 0;
    prev_counts = 0;
    prev_time = 0;
    this->resoultion = resoultion;
    
}
float Encoder::calculate_speed(){
    float current_time = milles();
    long new_counts = encoder_counts;
    int delta_counts = new_counts - prev_counts;
    float dt = current_time - prev_time;
    float speed = (seconds * milles/ resoultion)*(delta_counts/dt);
    prev_counts = new_counts;
    prev_time = current_time;
    return speed;
}



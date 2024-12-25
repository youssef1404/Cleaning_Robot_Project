#ifndef L298N_H
#define L298N_H

class L298N {
  private:
    int speed;
    int direction;
    uint8_t enable_pin, input1_pin, input2_pin;

  public:
    L298N(uint8_t enable_pin, uint8_t input1, uint8_t input2);
    void driver_init();
    void set_speed(int speed);
    void set_direction(int speed);
    void control_speed();
};

#endif
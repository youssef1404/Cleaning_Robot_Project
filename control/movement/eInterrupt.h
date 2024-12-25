#ifndef eInterrupt_H
#define eInterrupt_H

#define secondes_per_minute 60
#define milles 1000

class eInterrupt {
  private:
      long prev_counts= 0;
      float prev_time = 0;

  public:
      int pin_A, pin_B;
      eInterrupt(int pin_A, int pin_B, float resoultion);
      void Init();
      void ISR_A_routine();
      void ISR_B_routine();
      long encoder_counts = 0;
      float resoultion = 0;
      float calculate_speed();
};

#endif

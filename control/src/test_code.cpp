#include "l298n.h"
#include "Config.h"
#define speed_test 255
#define speed_stop 0

L298N l298n[]{L298N(enable_pin_1, input1_1, input2_1),
              L298N(enable_pin_2, input1_2, input2_2)};
void setup(){
    l298n[0].driver_init();
    l298n[1].driver_init();
}

void loop(){
    l298n[0].set_speed(speed_test);
    l298n[1].set_speed(speed_test);
    Serial.print("driver speed motor 1 set:");
    Serial.println(l298n[0].speed);
    Serial.print("driver speed motor 2 set:");
    Serial.println(l298n[1].speed);
    l298n[0].set_direction(speed_test);
    Serial.print("driver speed motor 2 set:");
    Serial.println(l298n[0].direction);
    l298n[1].set_direction(speed_test);
    Serial.print("driver speed motor 2 set:");
    Serial.println(l298n[1].direction);
    l298n[0].control_speed();
    l298n[1].control_speed();
}
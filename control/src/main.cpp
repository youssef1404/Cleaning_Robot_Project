#include "rover.h"

Rover Bor3y;

void setup() {
	Bor3y.setup();
}

void loop() {
	Serial.println("loop");
	Bor3y.loop();
}
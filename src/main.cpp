#include "MotorController.h"

MotorController motorController;

void setup() {
    motorController.init();
}

void loop() {
    motorController.loop();
}

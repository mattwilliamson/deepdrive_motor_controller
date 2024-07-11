#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "Config.h"

class MotorController {
public:
    MotorController();
    void init();
    void loop();

private:
    static HallSensor sensorFront;
    static HallSensor sensorBack;
    InlineCurrentSense currentSenseFront;
    InlineCurrentSense currentSenseBack;
    BLDCMotor motorFront;
    BLDCMotor motorBack;
    BLDCDriver3PWM driverFront;
    BLDCDriver3PWM driverBack;
    static float target_velocity;

    static void intFrontA();
    static void intFrontB();
    static void intFrontC();
    static void intBackA();
    static void intBackB();
    static void intBackC();

    void initMotorSettings(BLDCMotor& motor);
};

#endif // MOTOR_CONTROLLER_H

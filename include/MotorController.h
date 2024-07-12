#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "Config.h"

class MotorController {
public:
    MotorController(int hallPinA, int hallPinB, int hallPinC, int polePairs,
                    int driverPinU, int driverPinV, int driverPinW, int driverPinEn,
                    int currentSensePinU, int currentSensePinV);
    void init();
    void loop();
    void setTargetVelocity(float velocity);
    int64_t getTotalTicks();
    HallSensor sensor;
    InlineCurrentSense currentSense;
    BLDCMotor motor;
    BLDCDriver3PWM driver;

private:

    int64_t total_ticks;
    float target_velocity;

    void intA();
    void intB();
    void intC();

    static void intA_static(void* arg);
    static void intB_static(void* arg);
    static void intC_static(void* arg);

    void initHallSensor();
    void initCurrentSense();
    void initMotorSettings();
    void initDriver();
};

#endif // MOTOR_CONTROLLER_H

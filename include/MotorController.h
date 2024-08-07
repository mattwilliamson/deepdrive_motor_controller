#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H


#include <Arduino.h>
#include <SimpleFOC.h>

#include <TonePlayer.h>
#include <BLDCSpeaker.h>
#include <BLDCMotor.h>

#include "Config.h"
#include "RosoutLogger.hpp"

class MotorController {
public:
    MotorController(int hallPinA, int hallPinB, int hallPinC, int polePairs,
                    int driverPinU, int driverPinV, int driverPinW, int driverPinEn,
                    int currentSensePinU, int currentSensePinV, RosoutLogger* logger);
    // Copy assignment operator
    MotorController& operator=(const MotorController& other) = default;

    void initialize();
    void loop();
    void setTargetVelocity(float velocity);
    float getVelocity();
    float getCurrent();
    double getAngle();
    void playStartup();
    void playError();
    void playSuccess();
    
    HallSensor sensor;
    InlineCurrentSense currentSense;
    BLDCMotor motor;
    BLDCDriver3PWM driver;

    BLDCSpeaker speaker;
    TonePlayer player;

private:
    float target_velocity;
    RosoutLogger* logger;

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

#include "MotorController.h"

MotorController::MotorController(int hallPinA, int hallPinB, int hallPinC, int polePairs,
                                 int driverPinU, int driverPinV, int driverPinW, int driverPinEn,
                                 int currentSensePinU, int currentSensePinV)
    : sensor(hallPinA, hallPinB, hallPinC, polePairs),
      currentSense(0.01, (float)50.0, currentSensePinU, currentSensePinV),
      motor(polePairs),
      driver(driverPinU, driverPinV, driverPinW, driverPinEn),
      total_ticks(0),
      target_velocity(0)
{}

void MotorController::init() {
    delay(10000);

    initCurrentSense();
    initHallSensor();
    initMotorSettings();
    initDriver();

    motor.linkDriver(&driver);
    motor.linkCurrentSense(&currentSense);

    currentSense.linkDriver(&driver);

    Serial.begin(115200);

    motor.init();
    motor.initFOC();
}

void MotorController::loop() {
    motor.loopFOC();
    motor.move(target_velocity);
}

void MotorController::setTargetVelocity(float velocity) {
    target_velocity = velocity;
}

int64_t MotorController::getTotalTicks() {
    return total_ticks;
}

void MotorController::intA() {
    sensor.handleA();
    total_ticks++;
}

void MotorController::intB() {
    sensor.handleB();
    total_ticks++;
}

void MotorController::intC() {
    sensor.handleC();
    total_ticks++;
}

void MotorController::intA_static(void* arg) {
    static_cast<MotorController*>(arg)->intA();
}

void MotorController::intB_static(void* arg) {
    static_cast<MotorController*>(arg)->intB();
}

void MotorController::intC_static(void* arg) {
    static_cast<MotorController*>(arg)->intC();
}

void MotorController::initHallSensor() {
    sensor.init();
    attachInterruptArg(digitalPinToInterrupt(sensor.pinA), intA_static, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(sensor.pinB), intB_static, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(sensor.pinC), intC_static, this, CHANGE);
}

void MotorController::initCurrentSense() {
    currentSense.gain_a *= -1;
    currentSense.gain_b *= -1;
    currentSense.init();
}

void MotorController::initMotorSettings() {
    motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
    motor.velocity_index_search = VELOCITY_INDEX_SEARCH;

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::velocity;

    motor.PID_velocity.P = PID_VELOCITY_P;
    motor.PID_velocity.I = PID_VELOCITY_I;
    motor.PID_velocity.D = PID_VELOCITY_D;

    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.current_limit = CURRENT_LIMIT;

    motor.PID_velocity.output_ramp = OUTPUT_RAMP;
    motor.LPF_velocity.Tf = LPF_VELOCITY_TF;
    motor.velocity_limit = VELOCITY_LIMIT;
}

void MotorController::initDriver() {
    driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    driver.pwm_frequency = PWM_FREQUENCY;
    driver.init();
}

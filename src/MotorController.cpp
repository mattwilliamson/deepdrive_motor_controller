#include "MotorController.h"

MotorController::MotorController(int hallPinA, int hallPinB, int hallPinC, int polePairs,
                                 int driverPinU, int driverPinV, int driverPinW, int driverPinEn,
                                 int currentSensePinU, int currentSensePinV, RosoutLogger* logger)
    : sensor(hallPinA, hallPinB, hallPinC, polePairs),
      currentSense(0.01, (float)50.0, currentSensePinU, currentSensePinV),
      motor(polePairs),
      driver(driverPinU, driverPinV, driverPinW, driverPinEn),
      target_velocity(0),
      logger(logger)
{}


void MotorController::init() {
    // Current sense
    currentSense.gain_a *= CURRENT_SENSE_GAIN_A;
    currentSense.gain_b *= CURRENT_SENSE_GAIN_B;

    #ifndef FOC_CHECK_ENABLE
    // Detection disabled. Use specified values from config.
    currentSense.skip_align = true;
    #endif
    
    currentSense.init();

    // Hall Sensors
    sensor.init();
    attachInterruptArg(digitalPinToInterrupt(sensor.pinA), intA_static, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(sensor.pinB), intB_static, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(sensor.pinC), intC_static, this, CHANGE);
    motor.linkSensor(&sensor);

    motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
    motor.velocity_index_search = VELOCITY_INDEX_SEARCH;

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::velocity;

    motor.PID_velocity.P = PID_VELOCITY_P;
    motor.PID_velocity.I = PID_VELOCITY_I;
    motor.PID_velocity.D = PID_VELOCITY_D;

    motor.voltage_limit = VOLTAGE_LIMIT_INIT;
    motor.current_limit = CURRENT_LIMIT_INIT;

    motor.PID_velocity.output_ramp = PID_VELOCITY_RAMP;
    motor.LPF_velocity.Tf = LPF_VELOCITY_TF;
    motor.velocity_limit = VELOCITY_LIMIT;

    #ifndef FOC_CHECK_ENABLE
    // Detection disabled. Use specified values from config.
    motor.sensor_direction = SENSOR_DIRECTION;
    motor.zero_electric_angle = 0;
    #endif

    driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    driver.pwm_frequency = PWM_FREQUENCY;
    driver.init();

    motor.linkDriver(&driver);
    motor.linkCurrentSense(&currentSense);

    currentSense.linkDriver(&driver);

    // motor.useMonitoring(Serial);
    if (logger != nullptr) {
        motor.useMonitoring(*logger);
    }

    motor.init();
    motor.initFOC();

    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.current_limit = CURRENT_LIMIT;
}

void MotorController::loop() {
    motor.loopFOC();
    motor.move(target_velocity);
    sensor.update();
}

void MotorController::setTargetVelocity(float velocity) {
    target_velocity = velocity;
}

float MotorController::getVelocity() {
    return sensor.getVelocity();
}

float MotorController::getCurrent() {
    return motor.current_sp;
}

double MotorController::getAngle() {
    return sensor.getPreciseAngle();
}


void MotorController::intA() {
    sensor.handleA();
    // total_ticks++;
}

void MotorController::intB() {
    sensor.handleB();
    // total_ticks++;
}

void MotorController::intC() {
    sensor.handleC();
    // total_ticks++;
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
}

void MotorController::initCurrentSense() {

}

void MotorController::initMotorSettings() {
}

void MotorController::initDriver() {

}

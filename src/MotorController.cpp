#include "MotorController.h"

HallSensor MotorController::sensorFront(FRONT_HALL_PIN_A, FRONT_HALL_PIN_B, FRONT_HALL_PIN_C, 15);
HallSensor MotorController::sensorBack(BACK_HALL_PIN_A, BACK_HALL_PIN_B, BACK_HALL_PIN_C, 15);
float MotorController::target_velocity = 0;

MotorController::MotorController()
    : currentSenseFront(0.01, (float)50.0, FRONT_CURRENT_SENSE_PIN_U, FRONT_CURRENT_SENSE_PIN_V),
      currentSenseBack(0.01, (float)50.0, BACK_CURRENT_SENSE_PIN_U, BACK_CURRENT_SENSE_PIN_V),
      motorFront(MOTOR_FRONT_POLE_PAIRS),
      motorBack(MOTOR_BACK_POLE_PAIRS),
      driverFront(FRONT_DRIVER_PIN_U, FRONT_DRIVER_PIN_V, FRONT_DRIVER_PIN_W, FRONT_DRIVER_PIN_EN),
      driverBack(BACK_DRIVER_PIN_U, BACK_DRIVER_PIN_V, BACK_DRIVER_PIN_W, BACK_DRIVER_PIN_EN)
{}

void MotorController::init() {
    delay(10000);

    currentSenseFront.gain_a *= -1;
    currentSenseFront.gain_b *= -1;
    currentSenseBack.gain_a *= -1;
    currentSenseBack.gain_b *= -1;

    currentSenseFront.init();
    currentSenseBack.init();

    sensorFront.init();
    sensorBack.init();

    sensorFront.enableInterrupts(intFrontA, intFrontB, intFrontC);
    sensorBack.enableInterrupts(intBackA, intBackB, intBackC);

    initMotorSettings(motorFront);
    initMotorSettings(motorBack);

    driverFront.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    driverFront.pwm_frequency = PWM_FREQUENCY;
    driverFront.init();

    driverBack.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    driverBack.pwm_frequency = PWM_FREQUENCY;
    driverBack.init();

    motorFront.linkDriver(&driverFront);
    motorBack.linkDriver(&driverBack);

    motorFront.linkCurrentSense(&currentSenseFront);
    motorBack.linkCurrentSense(&currentSenseBack);

    currentSenseFront.linkDriver(&driverFront);
    currentSenseBack.linkDriver(&driverBack);

    Serial.begin(115200);

    motorFront.init();
    motorFront.initFOC();

    motorBack.init();
    motorBack.initFOC();
}

void MotorController::loop() {
    motorFront.loopFOC();
    motorBack.loopFOC();

    motorFront.move(target_velocity);
    motorBack.move(target_velocity);
}

void MotorController::intFrontA() {
    sensorFront.handleA();
}

void MotorController::intFrontB() {
    sensorFront.handleB();
}

void MotorController::intFrontC() {
    sensorFront.handleC();
}

void MotorController::intBackA() {
    sensorBack.handleA();
}

void MotorController::intBackB() {
    sensorBack.handleB();
}

void MotorController::intBackC() {
    sensorBack.handleC();
}

void MotorController::initMotorSettings(BLDCMotor& motor) {
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

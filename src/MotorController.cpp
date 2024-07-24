#include "MotorController.h"
#include <RTTTL.h>

// const char rtttl[] PROGMEM = "HarryPot:d=16,o=5,b=125:2p,8p,8b,8e.6,g6,8f#6,4e6,8b6,4a.6,4f#.6,8e.6,g6,8f#6,4d6,8f6,2b,8p,8b,8e.6,g6,8f#6,4e6,8b6,4d7,8c#7,4c7,8g#6,8c.7,b6,8a#6,4f#6,8g6,2e6,8p,8g6,4b6,8g6,4b6,8g6,4c7,8b6,4a#6,8f#6,8g.6,b6,8a#6,4a#,8b,2b6,8p";

// Song* song_startup = parseRTTL(rtttl);
// Song* song_error = parseRTTL("jamesbond:d=4,o=6,b=112:16c.5,32d.5,32d.5,16d.5,8d.5,16c.5,16c.5");
// Song* song_success = parseRTTL("jamesbond:d=4,o=6,b=112:16c.5,32d.5,32d.5,16d.5,8d.5,16c.5,16c.5");

MotorController::MotorController(int hallPinA, int hallPinB, int hallPinC,
                                 int polePairs, int driverPinU, int driverPinV,
                                 int driverPinW, int driverPinEn,
                                 int currentSensePinU, int currentSensePinV,
                                 RosoutLogger *logger)
    : sensor(hallPinA, hallPinB, hallPinC, polePairs),
      currentSense(0.01, (float)50.0, currentSensePinU, currentSensePinV),
      motor(polePairs), 
      driver(driverPinU, driverPinV, driverPinW, driverPinEn),
      target_velocity(0), 
      logger(logger), 
      speaker(&motor, VOLTAGE_LIMIT_TONE, TONE_VOLUME, TONE_NOTE_OFFSET) {}

void MotorController::playStartup() {
  // player.play(song_startup);
  // while (player.isPlaying()) {
    // player.loop();
    // vTaskDelay(1);
  // }
}

void MotorController::playError() {
  // player.play(song_error);
}

void MotorController::playSuccess() {
  // player.play(song_success);
}

void MotorController::initialize() {
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
  attachInterruptArg(digitalPinToInterrupt(sensor.pinA), intA_static, this,
                     CHANGE);
  attachInterruptArg(digitalPinToInterrupt(sensor.pinB), intB_static, this,
                     CHANGE);
  attachInterruptArg(digitalPinToInterrupt(sensor.pinC), intC_static, this,
                     CHANGE);
  motor.linkSensor(&sensor);

  motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
  motor.velocity_index_search = VELOCITY_INDEX_SEARCH;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::velocity;

  motor.PID_velocity.P = PID_VELOCITY_P;
  motor.PID_velocity.I = PID_VELOCITY_I;
  motor.PID_velocity.D = PID_VELOCITY_D;

  motor.voltage_limit = VOLTAGE_LIMIT_INIT;
  motor.current_limit = CURRENT_LIMIT;

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

  // Setup tone-lib ringtone player
  // player.attachSpeaker(&speaker);
  // Serial.println("player.play");
  // playStartup();
}


void MotorController::loop() {
  if (player.isPlaying()) {
    // Serial.println("player.isPlaying()");
    // Make sure the motor is stopped
    // if (sensor.getVelocity() == 0.0) {
    //   Serial.println("playing tone");

    //   // Set to open loop control for speaker
    //   motor.controller = MotionControlType::velocity_openloop;
    //   motor.voltage_limit = VOLTAGE_LIMIT_TONE;

    //   player.loop();
    // } else {
    //   Serial.println("stopping motor");
    //   target_velocity = 0;
    //   motor.loopFOC();
    //   motor.move(0.0);
    //   sensor.update(); 
    // }
  } else {
    // Serial.println("!player.isPlaying()");

    // TODO: Make some state machine or something to do this less frequently
    // motor.controller = MotionControlType::velocity;
    // motor.voltage_limit = VOLTAGE_LIMIT;

    motor.loopFOC();
    motor.move(target_velocity);
    sensor.update();
  }
}

void MotorController::setTargetVelocity(float velocity) {
  target_velocity = velocity;
}

float MotorController::getVelocity() { return sensor.getVelocity(); }

float MotorController::getCurrent() { return motor.current_sp; }

double MotorController::getAngle() { return sensor.getPreciseAngle(); }

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

void MotorController::intA_static(void *arg) {
  static_cast<MotorController *>(arg)->intA();
}

void MotorController::intB_static(void *arg) {
  static_cast<MotorController *>(arg)->intB();
}

void MotorController::intC_static(void *arg) {
  static_cast<MotorController *>(arg)->intC();
}

void MotorController::initHallSensor() {}

void MotorController::initCurrentSense() {}

void MotorController::initMotorSettings() {}

void MotorController::initDriver() {}

#include <Arduino.h>

/*
MKS ESP32 FOC Closed Loop Speed Control Example; Test Library：SimpleFOC 2.1.1; Test Hardware：MKS ESP32 FOC V1.0
Enter the "T+number" command in the serial port window to make the two motors rotate in closed loop.
For example, to have both motors turn at 10rad/s, enter: T10.
When using your own motor, please remember to modify the default number of pole pairs, the value in BLDCMotor()
The default power supply voltage set by the program is 12V.
Please remember to modify the values in voltage_power_supply and voltage_limit if you use other voltages for power supply.
The default PID is for the YT2804 motor. When using your own motor.
You need to modify the PID parameters to achieve better results.
 */
#include <SimpleFOC.h>

HallSensor sensor = HallSensor(18, 19, 15, 15);
HallSensor sensor1 = HallSensor(5, 23, 13, 15);

InlineCurrentSense current_sense0 = InlineCurrentSense(0.01, (float)50.0, 39, 36);
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, (float)50.0, 35, 34);


void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

void doA1(){sensor1.handleA();}
void doB1(){sensor1.handleB();}
void doC1(){sensor1.handleC();}


//  BLDCMotor(int pp, (optional R, KV))
//  - pp  - pole pair number
//  - R   - phase resistance value - optional
//  - KV  - motor KV rating [rpm/V] - optional

//Motor Parameters
BLDCMotor motor = BLDCMotor(15);                           //Modify the value in BLDMotor() according to the number of pole pairs of the selected motor
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(15);                          //Also modify the value in BLDMotor() here
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

#include <TonePlayer.h>
#include <BLDCSpeaker.h>
#include <BLDCMotor.h>
#include <RTTTL.h>

BLDCSpeaker speaker = BLDCSpeaker(&motor, 1.5, 10, -12);
BLDCSpeaker speaker1 = BLDCSpeaker(&motor1, 1.5, 10, -12);
TonePlayer player;
// const char rtttl[] PROGMEM = RTTTL_PINK_PANTHER;
const char rtttl[] PROGMEM = "HarryPot:d=16,o=5,b=125:2p,8p,8b,8e.6,g6,8f#6,4e6,8b6,4a.6,4f#.6,8e.6,g6,8f#6,4d6,8f6,2b,8p,8b,8e.6,g6,8f#6,4e6,8b6,4d7,8c#7,4c7,8g#6,8c.7,b6,8a#6,4f#6,8g6,2e6,8p,8g6,4b6,8g6,4b6,8g6,4c7,8b6,4a#6,8f#6,8g.6,b6,8a#6,4a#,8b,2b6,8p";

//Command Settings
float target_velocity = -2;                                //Enter "T+speed" in the serial monitor to make the two motors rotate in closed loop
Commander command = Commander(Serial);                    //For example, to make both motors rotate at a speed of 10rad/s, input "T10"
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  delay(10000);

  // pinMode(18, INPUT_PULLUP);
  // pinMode(19, INPUT_PULLUP);
  // pinMode(15, INPUT_PULLUP);

  // pinMode(5, INPUT_PULLUP);
  // pinMode(23, INPUT_PULLUP);
  // pinMode(13, INPUT_PULLUP);
    // sensor.velocity_max = 1000;
    // sensor1.velocity_max = 1000;

  current_sense0.gain_a *= -1;
  current_sense0.gain_b *= -1;
  current_sense1.gain_a *= -1;
  current_sense1.gain_b *= -1;

  current_sense0.init();
  current_sense1.init();




    sensor.init();
    sensor1.init();

    sensor.enableInterrupts(doA, doB, doC);
    sensor1.enableInterrupts(doA1, doB1, doC1);

    
  // motor1.torque_controller = TorqueControlType::foc_current; 
  // motor1.controller = MotionControlType::torque;
  // motor2.torque_controller = TorqueControlType::foc_current; 
  // motor2.controller = MotionControlType::torque;



 // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  motor1.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;
  motor1.velocity_index_search = 3;



  //Connecting Motor Objects to Sensor Objects
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //Supply Voltage Setting [V]
  driver.voltage_power_supply = 12;               //When using other supply voltages, modify the value of voltage_power_supply here
  driver.pwm_frequency = 20000;

  driver.init();

  driver1.voltage_power_supply = 12;              //Also modify the value of voltage_power_supply here
  driver1.pwm_frequency = 20000;

  driver1.init();
  //Connect the Motor and Driver Objects
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  motor.linkCurrentSense(&current_sense0);
  motor1.linkCurrentSense(&current_sense1);

  current_sense0.linkDriver(&driver);
  current_sense1.linkDriver(&driver1);
  
  //FOC Model Selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion Control Mode Settings
  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;


  //PID Settings
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0;

  motor1.PID_velocity.P = 0.2;
  motor1.PID_velocity.I = 1.0;
  motor1.PID_velocity.D = 0;
  //Maximum Motor Voltage Limit
  motor.voltage_limit = 3.0;                   //When using other power supply voltages, modify the value of voltage_limit here
  motor.current_limit = .25;
  motor1.voltage_limit = 3.0;                  //Also modify the value of voltage_limit here
  motor1.current_limit = .25;

  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  motor1.PID_velocity.output_ramp = 1000;
  
  //Speed Low-pass Filter Time Constant
  motor.LPF_velocity.Tf = 0.08; // .01 = 5ms (default)
  motor1.LPF_velocity.Tf = 0.08;

  //Maximum Speed Limit Settings
  motor.velocity_limit = 40;
  motor1.velocity_limit = 40;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  motor1.monitor_downsample = 100;
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  motor.monitor_downsample = 100;
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

  // current_sense0.skip_align = true;
  // current_sense1.skip_align = true;

  
  //Initialize the Motor
  Serial.println(F("Motor 1 init."));
  motor.init();
  motor.initFOC();

  Serial.println(F("---"));

  //Initialize FOC
  Serial.println(F("Motor 2 init."));
  motor1.init();
  motor1.initFOC();
  player.attachSpeaker(&speaker);
  player.attachSpeaker(&speaker1);

  Song* song = parseRTTL(rtttl);
  player.play(song);

  delay(1000);

  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
}



void print_current() {
  // PhaseCurrent_s currents0 = current_sense0.getPhaseCurrents();
  // float current_magnitude0 = current_sense0.getDCCurrent();
  // PhaseCurrent_s currents1 = current_sense1.getPhaseCurrents();
  // float current_magnitude1 = current_sense1.getDCCurrent();

  //   Serial.print(currents0.a*1000); // milli Amps
  // Serial.print("\t");
  // Serial.print(currents0.b*1000); // milli Amps
  // Serial.print("\t");
  // Serial.print(currents0.c*1000); // milli Amps
  // Serial.print("\t");
  // Serial.println(current_magnitude0*1000); // milli Amps
  // Serial.print(currents1.a*1000); // milli Amps
  // Serial.print("\t");
  // Serial.print(currents1.b*1000); // milli Amps
  // Serial.print("\t");
  // Serial.print(currents1.c*1000); // milli Amps
  // Serial.print("\t");
  // Serial.println(current_magnitude1*1000); // milli Amps
  // Serial.println();

}

void loop() {

    if(player.isPlaying()) {
      player.loop();
    } else {
  
    motor.voltage_limit = 12;                   //When using other power supply voltages, modify the value of voltage_limit here
    motor1.voltage_limit = 12;                  //Also modify the value of voltage_limit here

    motor.loopFOC();
    motor1.loopFOC();

    motor.move(target_velocity);
    motor1.move(target_velocity);

    command.run();

    // print_current();

    // motor1.monitor();
    // motor.monitor();
  }
}

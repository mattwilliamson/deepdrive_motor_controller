/**
 * @file Config.h
 * @brief Configuration file for the motor controller.
 * 
 * This file contains various configuration settings for the motor controller.
 * It defines constants for pin assignments, motor settings, voltage settings,
 * PID settings, LPF settings, speed limit, and other miscellaneous settings.
 */
#ifndef CONFIG_H
#define CONFIG_H

#define BOARD_DELAY 10000 /**< Delay in microseconds for board initialization. */
#define SERIAL_BAUD_RATE 115200 /**< Baud rate for serial communication. */

// Pub/sub timeouts
#define MESSAGE_RECEIVE_TIMEOUT 1000 /**< Start pinging the microros-agent if we haven't had a message for a while */
#define AGENT_PING_TIMEOUT 100 /**< Reboot the board if we can't reach the agent */
#define AGENT_PING_ATTEMPTS_REBOOT 200 /**< Reboot the board if we can't reach the agent for 200 * 100 milliseconds (20s) */

// Pin definitions
#define FRONT_HALL_PIN_A 18 /**< Pin number for front hall sensor A. */
#define FRONT_HALL_PIN_B 19 /**< Pin number for front hall sensor B. */
#define FRONT_HALL_PIN_C 15 /**< Pin number for front hall sensor C. */
#define BACK_HALL_PIN_A 5 /**< Pin number for back hall sensor A. */
#define BACK_HALL_PIN_B 23 /**< Pin number for back hall sensor B. */
#define BACK_HALL_PIN_C 13 /**< Pin number for back hall sensor C. */

#define FRONT_DRIVER_PIN_U 32 /**< Pin number for front driver U. */
#define FRONT_DRIVER_PIN_V 33 /**< Pin number for front driver V. */
#define FRONT_DRIVER_PIN_W 25 /**< Pin number for front driver W. */
#define FRONT_DRIVER_PIN_EN 22 /**< Pin number for front driver enable. */

#define BACK_DRIVER_PIN_U 26 /**< Pin number for back driver U. */
#define BACK_DRIVER_PIN_V 27 /**< Pin number for back driver V. */
#define BACK_DRIVER_PIN_W 14 /**< Pin number for back driver W. */
#define BACK_DRIVER_PIN_EN 12 /**< Pin number for back driver enable. */

#define FRONT_CURRENT_SENSE_PIN_U 39 /**< Pin number for front current sense U. */
#define FRONT_CURRENT_SENSE_PIN_V 36 /**< Pin number for front current sense V. */
#define BACK_CURRENT_SENSE_PIN_U 35 /**< Pin number for back current sense U. */
#define BACK_CURRENT_SENSE_PIN_V 34 /**< Pin number for back current sense V. */

// Motor settings
#define MOTOR_FRONT_POLE_PAIRS 15 /**< Number of pole pairs for the front motor. */
#define MOTOR_BACK_POLE_PAIRS 15 /**< Number of pole pairs for the back motor. */

// Voltage settings
#define VOLTAGE_SENSOR_ALIGN 3 /**< Voltage sensor alignment value. */
#define VOLTAGE_POWER_SUPPLY 12 /**< Power supply voltage value. */
#define VOLTAGE_LIMIT_INIT VOLTAGE_SENSOR_ALIGN /**< Voltage limit value. */
#define CURRENT_LIMIT_INIT 0.5 /**< Current limit value. */
#define VOLTAGE_LIMIT 12.0 /**< Voltage limit value. */
#define CURRENT_LIMIT 0.5 /**< Current limit value. */
#define PWM_FREQUENCY 20000 /**< PWM frequency value. */

#define VOLTAGE_LIMIT_TONE 1.5 /**< Voltage limit value for tone player. */
#define TONE_VOLUME 50 /**< Volume value for tone player. */
#define TONE_NOTE_OFFSET -12 /**< Note offset value for tone player (-12 is down an octave). */

// PID settings
#define PID_VELOCITY_P 0.25 /**< Proportional gain for velocity PID controller. */
#define PID_VELOCITY_I 1.0 /**< Integral gain for velocity PID controller. */
#define PID_VELOCITY_D 0 /**< Derivative gain for velocity PID controller. */
#define PID_VELOCITY_RAMP 1000 /**< Output ramp value for velocity PID controller. */

// LPF settings
#define LPF_VELOCITY_TF 0.08 /**< Time constant for velocity low-pass filter. */

// Speed limit
#define VELOCITY_LIMIT 20 /**< Maximum velocity limit. */

// Other settings
#define VELOCITY_INDEX_SEARCH 3 /**< Index search value for velocity control. */

#define PUBLISH_ODOM_HZ 10 /**< Frequency for publishing odometry data. */

#define MOTOR_LOOP_PERIOD 1 /**< Period for motor control loop. in microseconds */
#define MICRO_ROS_LOOP_PERIOD 10 /**< Period for micro-ROS control loop. in microseconds */

// #define SWAP_LEFT_SIDE_MOTORS /**< Flag to swap left side front/back motors. */
#define SWAP_RIGHT_SIDE_MOTORS /**< Flag to swap right side front/back motors. */

// #define INVERT_FRONT_MOTOR /**< Flag to invert left motor direction. */
// #define INVERT_BACK_MOTOR /**< Flag to invert right motor direction. */

// Ping the agent at startup or reboot
#define UROS_TIMEOUT_STARTUP 1000 /**< Timeout value for UROS communication (ms). */
#define UROS_ATTEMPTS_STARTUP 10 /**< Number of attempts for UROS communication. */

// Ping the agent periodically
#define UROS_TIMEOUT_PERIODIC 10 /**< Timeout value for UROS communication (ms). */
#define UROS_ATTEMPTS_PERIODIC 3 /**< Number of attempts for UROS communication. */

#define FOC_CHECK_ENABLE /**< Flag to enable FOC check., which checks pole pairs, direction and electrical angle. Will log to /rosout */
// TODO: Make this configurable for each different motor
#define SENSOR_DIRECTION Direction::CCW /**< Direction of the front sensor. */

#define CURRENT_SENSE_GAIN_A -1 /**< Gain value for current sense A. */
#define CURRENT_SENSE_GAIN_B -1 /**< Gain value for current sense B. */

#endif // CONFIG_H

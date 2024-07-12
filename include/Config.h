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
#define VOLTAGE_LIMIT 3.0 /**< Voltage limit value. */
#define CURRENT_LIMIT 0.25 /**< Current limit value. */
#define PWM_FREQUENCY 20000 /**< PWM frequency value. */

// PID settings
#define PID_VELOCITY_P 0.2 /**< Proportional gain for velocity PID controller. */
#define PID_VELOCITY_I 1.0 /**< Integral gain for velocity PID controller. */
#define PID_VELOCITY_D 0 /**< Derivative gain for velocity PID controller. */
#define PID_VELOCITY_RAMP 1000 /**< Output ramp value for velocity PID controller. */

// LPF settings
#define LPF_VELOCITY_TF 0.08 /**< Time constant for velocity low-pass filter. */

// Speed limit
#define VELOCITY_LIMIT 40 /**< Maximum velocity limit. */

// Other settings
#define VELOCITY_INDEX_SEARCH 3 /**< Index search value for velocity control. */

#endif // CONFIG_H

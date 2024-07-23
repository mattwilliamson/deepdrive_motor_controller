#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64.h>

#include "Config.h"
#include "MotorController.h"
#include "RosoutLogger.hpp"
#include "Settings.h"

// Error handle loop
void error_loop() {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Error");
  }
}

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      Serial.println("error code: " + String(temp_rc));                        \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc !=
// RCL_RET_OK)){printf("Failed status on line %d: %d.
// Aborting.\n",__LINE__,(int)temp_rc); return 1;}} #define RCSOFTCHECK(fn) {
// rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on
// line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

RosoutLogger *logger;

rcl_publisher_t front_publisher;
rcl_subscription_t front_subscriber;
std_msgs__msg__Float64 front_pub_msg;
std_msgs__msg__Float32 front_sub_msg;

rcl_publisher_t back_publisher;
rcl_subscription_t back_subscriber;
std_msgs__msg__Float64 back_pub_msg;
std_msgs__msg__Float32 back_sub_msg;

rcl_timer_t timer_odom;

MotorController *frontMotorController;
MotorController *backMotorController;

uint32_t last_message_time = 0;

void front_subscription_callback(const void *msgin) {
  last_message_time = millis();
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  frontMotorController->setTargetVelocity(msg->data);
}

void back_subscription_callback(const void *msgin) {
  last_message_time = millis();
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  backMotorController->setTargetVelocity(msg->data);
}

void timer_odom_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    front_pub_msg.data = frontMotorController->getAngle();
    RCSOFTCHECK(rcl_publish(&front_publisher, &front_pub_msg, NULL));

    back_pub_msg.data = backMotorController->getAngle();
    RCSOFTCHECK(rcl_publish(&back_publisher, &back_pub_msg, NULL));
  }
}

void swapMotors(Settings &settings) {
  if (settings.getSide() == Side::LEFT) {
#ifdef SWAP_LEFT_SIDE_MOTORS
    MotorController *temp = frontMotorController;
    frontMotorController = backMotorController;
    backMotorController = temp;
#endif
  } else {
#ifdef SWAP_RIGHT_SIDE_MOTORS
    MotorController *temp = frontMotorController;
    frontMotorController = backMotorController;
    backMotorController = temp;
#endif
  }
}

void vTaskMotorFront(void *pvParameters) {
  frontMotorController->initialize();

  // TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    frontMotorController->loop();
    vTaskDelay(MOTOR_LOOP_PERIOD * portTICK_PERIOD_MS / 1000);
  }
}

void vTaskMotorBack(void *pvParameters) {
  backMotorController->initialize();

  // TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    backMotorController->loop();
    vTaskDelay(MOTOR_LOOP_PERIOD * portTICK_PERIOD_MS / 1000);
  }
}

void vTaskMicroROS(void *pvParameters) {
  const TickType_t xDelay = MOTOR_LOOP_PERIOD * portTICK_PERIOD_MS / 1000;
  while (true) {
    // RCCHECK(rclc_executor_spin_some(&executor));
    // TODO: Should this go smaller? Will it block the kernel?
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    vTaskDelay(MOTOR_LOOP_PERIOD * portTICK_PERIOD_MS / 1000);
  }
}

bool pingAgent() {
    return rmw_uros_ping_agent(UROS_TIMEOUT_PERIODIC, 1) == RMW_RET_OK;
}

void vTaskPing(void *pvParameters) {
  last_message_time = millis();
  const TickType_t xDelay = pdMS_TO_TICKS(MESSAGE_RECEIVE_TIMEOUT / 2);

  static bool connected = false;
  static uint32_t ping_failures = 0;

  while (true) {
    // If we don't get messages and the agent is not available, stop the motors
    uint32_t elapsed = millis() - last_message_time;

    if (elapsed > MESSAGE_RECEIVE_TIMEOUT) {
      // We lost communication with the micro ros agent. Stop the motors.
      frontMotorController->setTargetVelocity(0);
      backMotorController->setTargetVelocity(0);

      Serial.println("Error: No message received for " + String(elapsed) +
                     "ms");

      if (!pingAgent()) {
        Serial.println("ping failed");

        connected = false;
        ping_failures++;

        if (ping_failures >= AGENT_PING_ATTEMPTS_REBOOT) {
          // Reboot the board
          Serial.println(
              "Critical: Agent not available after retries. Rebooting");
          abort();
        }
      } else {
        // Initial ping successful to agent
        if (!connected) {
          Serial.println("initial connection");

          // Synchronize time with the agent
          rmw_uros_sync_session(5000);

          connected = true;
          ping_failures = 0;

          backMotorController->playSuccess();
          // frontMotorController->playSuccess();
        }
      }
    }

    vTaskDelay(xDelay);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ; // Wait for serial port to connect (needed for some boards)

  // Read which side this is from EEPROM or wait for it to be set over serial
  Settings &settings = Settings::getInstance();

  // Start micro ros
  set_microros_serial_transports(Serial);
  delay(2000);

  // Wait for agent to be ready
  // rmw_ret_t error_code =
  //     rmw_uros_ping_agent(UROS_TIMEOUT_STARTUP, UROS_ATTEMPTS_STARTUP);
  // if (error_code != RMW_RET_OK) {
  //   Serial.println("Error: Agent not available");
  //   // Reboot the board
  //   abort();
  // }


  // Give some time in case we are reflashing the board
  // Turn this off for now, since we added the ping to wait for the agent
  // delay(BOARD_DELAY);

#ifdef INVERT_FRONT_MOTOR
  // Swap 2 pins
  frontMotorController = new MotorController(
      RONT_HALL_PIN_B, FRONT_HALL_PIN_A, FRONT_HALL_PIN_C,
      MOTOR_FRONT_POLE_PAIRS, FRONT_DRIVER_PIN_U, FRONT_DRIVER_PIN_V,
      FRONT_DRIVER_PIN_W, FRONT_DRIVER_PIN_EN, FRONT_CURRENT_SENSE_PIN_U,
      FRONT_CURRENT_SENSE_PIN_V, logger);
#else
  frontMotorController = new MotorController(
      FRONT_HALL_PIN_A, FRONT_HALL_PIN_B, FRONT_HALL_PIN_C,
      MOTOR_FRONT_POLE_PAIRS, FRONT_DRIVER_PIN_U, FRONT_DRIVER_PIN_V,
      FRONT_DRIVER_PIN_W, FRONT_DRIVER_PIN_EN, FRONT_CURRENT_SENSE_PIN_U,
      FRONT_CURRENT_SENSE_PIN_V, logger);
#endif

#ifdef INVERT_BACK_MOTOR
  // Swap 2 pins
  backMotorController = new MotorController(
      BACK_HALL_PIN_B, BACK_HALL_PIN_A, BACK_HALL_PIN_C, MOTOR_BACK_POLE_PAIRS,
      BACK_DRIVER_PIN_U, BACK_DRIVER_PIN_V, BACK_DRIVER_PIN_W,
      BACK_DRIVER_PIN_EN, BACK_CURRENT_SENSE_PIN_U, BACK_CURRENT_SENSE_PIN_V,
      logger);
#else
  backMotorController = new MotorController(
      BACK_HALL_PIN_A, BACK_HALL_PIN_B, BACK_HALL_PIN_C, MOTOR_BACK_POLE_PAIRS,
      BACK_DRIVER_PIN_U, BACK_DRIVER_PIN_V, BACK_DRIVER_PIN_W,
      BACK_DRIVER_PIN_EN, BACK_CURRENT_SENSE_PIN_U, BACK_CURRENT_SENSE_PIN_V,
      logger);
#endif

  // Switch the front and back motors if necessary
  swapMotors(settings);

  // Start the tasks
  xTaskCreate(vTaskMotorFront, "MotorTaskFront", 10000, NULL, 16, NULL);
  // xTaskCreate(vTaskMotorBack, "MotorTaskBack", 10000, NULL, 16, NULL);
  // xTaskCreate(vTaskPing, "PingTask", 10000, NULL, 4, NULL);

  // Initialize micro ros

  // // Alloc memory
  // allocator = rcl_get_default_allocator();
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // RCCHECK(rclc_node_init_default(&node, "deepdrive_motor_controller_node",
  //                                settings.getNamespace(), &support));

  // // Front motor ROS setup
  // RCCHECK(rclc_publisher_init_default(
  //     &front_publisher, &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "front/angle"));
  // RCCHECK(rclc_subscription_init_default(
  //     &front_subscriber, &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "front/vel/cmd"));

  // // Back motor ROS setup
  // RCCHECK(rclc_publisher_init_default(
  //     &back_publisher, &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "back/angle"));
  // RCCHECK(rclc_subscription_init_default(
  //     &back_subscriber, &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "back/vel/cmd"));

  // // TODO: Publish velocity output

  // // Timer setup
  // RCCHECK(rclc_timer_init_default(&timer_odom, &support,
  //                                 RCL_S_TO_NS(1) / PUBLISH_ODOM_HZ,
  //                                 timer_odom_callback));

  // // Executor setup
  // RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));
  // RCCHECK(rclc_executor_add_subscription(
  //     &executor, &front_subscriber, &front_sub_msg, front_subscription_callback,
  //     ON_NEW_DATA));
  // RCCHECK(
  //     rclc_executor_add_subscription(&executor, &back_subscriber, &back_sub_msg,
  //                                    back_subscription_callback, ON_NEW_DATA));

  // // Setup the ROS logger
  // // logger = new RosoutLogger(&node, &support);
  // // logger->println("ROS logger initialized");

  // xTaskCreate(vTaskMicroROS, "MicroROSTask", 10000, NULL, 1, NULL);
}

void loop() {}

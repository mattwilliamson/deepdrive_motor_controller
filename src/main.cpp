#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int64.h>

#include "Config.h"
#include "RobotSide.h"
#include "MotorController.h"

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

RobotSide& robotSide = RobotSide::getInstance();

MotorController frontMotorController(FRONT_HALL_PIN_A, FRONT_HALL_PIN_B, FRONT_HALL_PIN_C, MOTOR_FRONT_POLE_PAIRS,
                                     FRONT_DRIVER_PIN_U, FRONT_DRIVER_PIN_V, FRONT_DRIVER_PIN_W, FRONT_DRIVER_PIN_EN,
                                     FRONT_CURRENT_SENSE_PIN_U, FRONT_CURRENT_SENSE_PIN_V);

MotorController backMotorController(BACK_HALL_PIN_A, BACK_HALL_PIN_B, BACK_HALL_PIN_C, MOTOR_BACK_POLE_PAIRS,
                                    BACK_DRIVER_PIN_U, BACK_DRIVER_PIN_V, BACK_DRIVER_PIN_W, BACK_DRIVER_PIN_EN,
                                    BACK_CURRENT_SENSE_PIN_U, BACK_CURRENT_SENSE_PIN_V);

rcl_publisher_t front_publisher;
rcl_subscription_t front_subscriber;
std_msgs__msg__Int64 front_pub_msg;
std_msgs__msg__Float32 front_sub_msg;

rcl_publisher_t back_publisher;
rcl_subscription_t back_subscriber;
std_msgs__msg__Int64 back_pub_msg;
std_msgs__msg__Float32 back_sub_msg;

rcl_timer_t timer;
rclc_executor_t executor;

void front_subscription_callback(const void* msgin) {
    const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
    frontMotorController.setTargetVelocity(msg->data);
}

void back_subscription_callback(const void* msgin) {
    const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
    backMotorController.setTargetVelocity(msg->data);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        front_pub_msg.data = frontMotorController.getTotalTicks();
        RCSOFTCHECK(rcl_publish(&front_publisher, &front_pub_msg, NULL));

        back_pub_msg.data = backMotorController.getTotalTicks();
        RCSOFTCHECK(rcl_publish(&back_publisher, &back_pub_msg, NULL));
    }
}


void setup() {
    // Give some time in case we are reflashing the board
    delay(10000);

    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect (needed for some boards)

    set_microros_serial_transports(Serial);
    // delay(2000);

    frontMotorController.init();
    backMotorController.init();

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, robotSide.getNodeName(), robotSide.getNamespace(), &support));

    // Front motor ROS setup
    RCCHECK(rclc_publisher_init_default(
        &front_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "front_ticks"));

    RCCHECK(rclc_subscription_init_default(
        &front_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "front_motor_velocity"));

    // Back motor ROS setup
    RCCHECK(rclc_publisher_init_default(
        &back_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "back_ticks"));

    RCCHECK(rclc_subscription_init_default(
        &back_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "back_velocity"));

    // Timer setup
    const unsigned int timer_timeout = 1000; // 1 second
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Executor setup
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &front_subscriber, &front_sub_msg, front_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &back_subscriber, &back_sub_msg, back_subscription_callback, ON_NEW_DATA));
}

void loop() {
    frontMotorController.loop();
    backMotorController.loop();

    // Spin the ROS executor
    // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    rclc_executor_spin_some(&executor, 10);
}

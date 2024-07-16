#ifndef ROSOUT_LOGGER_HPP
#define ROSOUT_LOGGER_HPP

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl_interfaces/msg/log.h>
#include <rcl/logging_rosout.h>

#include <Arduino.h>

#define ARRAY_LEN 200

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("error code: " + String(temp_rc)); error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

class RosoutLogger : public Print {
public:
    RosoutLogger(rcl_node_t* node, rclc_support_t* support);
    ~RosoutLogger();

    void publish_log_message(const char* message, uint8_t level = rcl_interfaces__msg__Log__INFO);

    // Print interface methods
    size_t print(const char* message);
    size_t println(const char* message);

    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buffer, size_t size) override;

private:
    rcl_publisher_t publisher;
    rcl_interfaces__msg__Log log_msg;
    int counter = 0;
    char buffer[ARRAY_LEN];
    size_t buffer_index;

    void initialize_publisher(rcl_node_t* node);
    void error_loop();
};

#endif // ROSOUT_LOGGER_HPP

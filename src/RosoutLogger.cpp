#include "RosoutLogger.hpp"

RosoutLogger::RosoutLogger(rcl_node_t* node, rclc_support_t* support)
: publisher(rcl_get_zero_initialized_publisher()), buffer_index(0)
{
    initialize_publisher(node);

    log_msg.level = rcl_interfaces__msg__Log__INFO;
    log_msg.name.data = (char*)malloc(ARRAY_LEN * sizeof(char));
    log_msg.name.size = 0;
    log_msg.name.capacity = ARRAY_LEN;
    log_msg.msg.data = (char*)malloc(ARRAY_LEN * sizeof(char));
    log_msg.msg.size = 0;
    log_msg.msg.capacity = ARRAY_LEN;
}

RosoutLogger::~RosoutLogger()
{
    RCCHECK(rcl_publisher_fini(&publisher, nullptr));
    free(log_msg.name.data);
    free(log_msg.msg.data);
}

void RosoutLogger::initialize_publisher(rcl_node_t* node)
{
    RCCHECK(rclc_publisher_init(
        &publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
        "/rosout",
        &rcl_qos_profile_rosout_default));
}

void RosoutLogger::publish_log_message(const char* message, uint8_t level)
{
    snprintf(log_msg.name.data, ARRAY_LEN, "rosout_logger_node");
    log_msg.name.size = strlen(log_msg.name.data);
    snprintf(log_msg.msg.data, ARRAY_LEN, "%s #%d", message, counter++);
    log_msg.msg.size = strlen(log_msg.msg.data);
    log_msg.level = level;

    RCSOFTCHECK(rcl_publish(&publisher, &log_msg, nullptr));
}

void RosoutLogger::error_loop()
{
    while (1) {
        delay(100);
    }
}

size_t RosoutLogger::print(const char* message)
{
    size_t len = strlen(message);
    if (buffer_index + len >= ARRAY_LEN) {
        len = ARRAY_LEN - buffer_index - 1; // Prevent overflow
    }
    strncpy(&buffer[buffer_index], message, len);
    buffer_index += len;
    buffer[buffer_index] = '\0'; // Null-terminate buffer
    return len;
}

size_t RosoutLogger::println(const char* message)
{
    size_t len = print(message);
    publish_log_message(buffer);
    buffer_index = 0; // Reset buffer index after publishing
    return len + 1; // Include newline character in the length
}

size_t RosoutLogger::write(uint8_t c)
{
    char buf[2] = { (char)c, '\0' };
    return print(buf);
}

size_t RosoutLogger::write(const uint8_t* buffer, size_t size)
{
    char* buf = (char*)malloc(size + 1);
    memcpy(buf, buffer, size);
    buf[size] = '\0';
    size_t len = print(buf);
    free(buf);
    return len;
}

#ifndef INITSERIAL_H
#define INITSERIAL_H
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

bool initSerial(serial::Serial& serial,
                const std::string& port_name,
                uint32_t baudrate,
                rclcpp::Logger logger);

#endif

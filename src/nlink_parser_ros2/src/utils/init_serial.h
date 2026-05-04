#ifndef INITSERIAL_H
#define INITSERIAL_H
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

// Open `serial` on `port_name` at `baudrate`. On failure returns false.
//
// `quiet`: when true, log failures at DEBUG instead of ERROR. Use this from
// the reconnect loop so a temporary disconnect does not flood /rosout with
// duplicate ERROR lines on every retry — the caller is expected to issue
// its own throttled WARN summary.
bool initSerial(serial::Serial& serial,
                const std::string& port_name,
                uint32_t baudrate,
                rclcpp::Logger logger,
                bool quiet = false);

#endif

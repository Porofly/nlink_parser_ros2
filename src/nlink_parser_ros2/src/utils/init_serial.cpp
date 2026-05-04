#include "init_serial.h"

bool initSerial(serial::Serial& serial,
                const std::string& port_name,
                uint32_t baudrate,
                rclcpp::Logger logger,
                bool quiet)
{
  try {
    serial.setPort(port_name);
    serial.setBaudrate(baudrate);

    if (!quiet) {
      RCLCPP_INFO(logger, "Opening serial port: %s @ %u baud", port_name.c_str(), baudrate);
    }

    auto timeout = serial::Timeout::simpleTimeout(10);
    serial.setTimeout(timeout);
    serial.open();

    if (serial.isOpen()) {
      if (!quiet) {
        RCLCPP_INFO(logger, "Serial port opened successfully");
      }
      return true;
    }
    if (quiet) {
      RCLCPP_DEBUG(logger, "Failed to open serial port (quiet)");
    } else {
      RCLCPP_ERROR(logger, "Failed to open serial port");
    }
    return false;
  } catch (const std::exception& e) {
    if (quiet) {
      RCLCPP_DEBUG(logger, "Serial open exception (quiet): %s", e.what());
    } else {
      RCLCPP_ERROR(logger, "Serial open exception: %s", e.what());
    }
    return false;
  }
}

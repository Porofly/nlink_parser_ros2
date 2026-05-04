#include "init_serial.h"

bool initSerial(serial::Serial& serial,
                const std::string& port_name,
                uint32_t baudrate,
                rclcpp::Logger logger)
{
  try
  {
    serial.setPort(port_name);
    serial.setBaudrate(baudrate);

    RCLCPP_INFO(logger, "Opening serial port: %s @ %u baud", port_name.c_str(), baudrate);

    auto timeout = serial::Timeout::simpleTimeout(10);
    serial.setTimeout(timeout);
    serial.open();

    if (serial.isOpen())
    {
      RCLCPP_INFO(logger, "Serial port opened successfully");
      return true;
    }
    RCLCPP_ERROR(logger, "Failed to open serial port");
    return false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Serial open exception: %s", e.what());
    return false;
  }
}

#include "init_serial.h"
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>
#include "rclcpp/rclcpp.hpp"
#include <fstream>

bool initSerial(serial::Serial& serial, const std::string& param_file_path)
{
  auto logger = rclcpp::get_logger("initSerial");

  try
  {
    std::ifstream file_check(param_file_path);
    if (!file_check.good()) {
      RCLCPP_ERROR(logger, "Parameter file not found: %s", param_file_path.c_str());
      return false;
    }

    YAML::Node config = YAML::LoadFile(param_file_path);

    if (!config["port_config"]) {
      RCLCPP_ERROR(logger, "Missing 'port_config' section in YAML file");
      return false;
    }

    YAML::Node port_config = config["port_config"];

    std::string port_name = port_config["port_name"]
        ? port_config["port_name"].as<std::string>()
        : "/dev/ttyUSB1";

    std::string baudrate_str = port_config["baudrate"]
        ? port_config["baudrate"].as<std::string>()
        : "921600";

    uint32_t baudrate = static_cast<uint32_t>(std::stoi(baudrate_str));

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
    else
    {
      RCLCPP_ERROR(logger, "Failed to open serial port");
      return false;
    }
  }
  catch (const YAML::BadFile& e) {
    RCLCPP_ERROR(logger, "YAML file not found: %s", e.what());
    return false;
  }
  catch (const YAML::ParserException& e) {
    RCLCPP_ERROR(logger, "YAML parsing error: %s", e.what());
    return false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Exception: %s", e.what());
    return false;
  }
}

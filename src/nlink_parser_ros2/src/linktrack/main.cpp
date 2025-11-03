#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("linktrack_main"),
                 "Usage: %s <param_file_path>", argv[0]);
    return EXIT_FAILURE;
  }

  serial::Serial serial;
  if (!initSerial(serial, std::string(argv[1]))) {
    RCLCPP_ERROR(rclcpp::get_logger("linktrack_main"),
                 "Failed to initialize serial port");
    return EXIT_FAILURE;
  }

  NProtocolExtracter protocol_extraction;
  auto linktrack_node = std::make_shared<linktrack::Init>(&protocol_extraction, &serial);

  rclcpp::spin(linktrack_node);
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}

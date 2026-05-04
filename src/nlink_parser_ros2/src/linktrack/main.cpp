#include "init.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<linktrack::Init>();
  if (!node->ok()) {
    RCLCPP_FATAL(node->get_logger(), "Linktrack node failed to initialize");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(),
                 "Unhandled exception in spin: %s", e.what());
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

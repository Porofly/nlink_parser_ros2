#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <thread>

#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe2.hpp>

#include "nlink_unpack/nlink_utils.h"
#include "protocol_extracter/nprotocol_extracter.h"

using nodeframe2   = nlink_parser_ros2_interfaces::msg::LinktrackNodeframe2;

namespace linktrack
{
  class Init : public rclcpp::Node
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial);
    ~Init();

  private:
    void initNodeFrame2(NProtocolExtracter *protocol_extraction);
    void startSerialReadThread();

    serial::Serial *serial_;
    NProtocolExtracter *protocol_extraction_;

    std::thread serial_thread_;

    rclcpp::Publisher<nodeframe2>  ::SharedPtr pub_node_frame2_;
  };
} // namespace linktrack

#endif // LINKTRACKINIT_H
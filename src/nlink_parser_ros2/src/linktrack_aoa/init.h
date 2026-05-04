#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include "../linktrack/protocols.h"
#include "nlink_protocol.h"

#include "std_msgs/msg/string.hpp"
#include <nlink_parser_ros2_interfaces/msg/linktrack_aoa_nodeframe0.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe0.hpp>

#include "protocol_extracter/nprotocol_extracter.h"
#include "protocol_manager.h"

namespace linktrack_aoa
{
  class Init : public rclcpp::Node
  {
  public:
    Init();
    bool ok() const { return serial_.isOpen(); }

  private:
    serial::Serial serial_;
    std::unique_ptr<NProtocolExtracter> protocol_extraction_;
    ProtocolManager protocol_manager_;
    std::string frame_id_;

    rclcpp::TimerBase::SharedPtr serial_read_timer_;
    void serialReadTimer();
    void initDataTransmission();
    void initNodeFrame0();
    void InitAoaNodeFrame0();

    rclcpp::Publisher<nlink_parser_ros2_interfaces::msg::LinktrackNodeframe0>::SharedPtr pub_node_frame0_;
    rclcpp::Publisher<nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0>::SharedPtr pub_aoa_node_frame0_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
  };
} // namespace linktrack_aoa

#endif // LINKTRACKAOAINIT_H

#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include "../linktrack/protocols.h"
#include "nlink_protocol.h"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
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
    bool ok() const { return initialized_; }

  private:
    serial::Serial serial_;
    std::string port_name_;
    uint32_t baudrate_{0};
    bool initialized_{false};
    rclcpp::Time last_reconnect_attempt_;
    rclcpp::Time last_frame_time_;
    double frame_timeout_warn_sec_{1.0};
    double frame_timeout_error_sec_{5.0};

    std::unique_ptr<NProtocolExtracter> protocol_extraction_;
    ProtocolManager protocol_manager_;
    std::string frame_id_;

    rclcpp::TimerBase::SharedPtr serial_read_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    bool tryOpenSerial(bool quiet = false);
    void serialReadTimer();
    void diagnosticsTimer();
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& params);
    void noteFrameReceived() { last_frame_time_ = this->now(); }
    void initDataTransmission();
    void initNodeFrame0();
    void InitAoaNodeFrame0();

    rclcpp::Publisher<nlink_parser_ros2_interfaces::msg::LinktrackNodeframe0>::SharedPtr pub_node_frame0_;
    rclcpp::Publisher<nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0>::SharedPtr pub_aoa_node_frame0_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
  };
} // namespace linktrack_aoa

#endif // LINKTRACKAOAINIT_H

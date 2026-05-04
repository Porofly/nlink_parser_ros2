#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlink_parser_ros2_interfaces/msg/linktrack_anchorframe0.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe0.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe1.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe2.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe3.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe5.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe6.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_tagframe0.hpp>

#include "nlink_unpack/nlink_utils.h"
#include "protocol_extracter/nprotocol_extracter.h"
#include "protocol_manager.h"

using anchorframe0 = nlink_parser_ros2_interfaces::msg::LinktrackAnchorframe0;
using tagframe0 = nlink_parser_ros2_interfaces::msg::LinktrackTagframe0;
using nodeframe0 = nlink_parser_ros2_interfaces::msg::LinktrackNodeframe0;
using nodeframe1 = nlink_parser_ros2_interfaces::msg::LinktrackNodeframe1;
using nodeframe2 = nlink_parser_ros2_interfaces::msg::LinktrackNodeframe2;
using nodeframe3 = nlink_parser_ros2_interfaces::msg::LinktrackNodeframe3;
using nodeframe5 = nlink_parser_ros2_interfaces::msg::LinktrackNodeframe5;
using nodeframe6 = nlink_parser_ros2_interfaces::msg::LinktrackNodeframe6;

class NProtocolExtracter;
namespace linktrack
{
  class Init  : public rclcpp::Node
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

    std::unique_ptr<NProtocolExtracter> protocol_extraction_;
    ProtocolManager protocol_manager_;
    std::string frame_id_;

    bool tryOpenSerial(bool quiet = false);
    void initDataTransmission();
    void serialReadTimer();
    void onParameterEvent(const std::vector<rclcpp::Parameter>& params);
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& params);
    void diagnosticsTimer();
    void noteFrameReceived() { last_frame_time_ = this->now(); }

    void initAnchorFrame0();
    void initTagFrame0();
    void initNodeFrame0();
    void initNodeFrame1();
    void initNodeFrame2();
    void initNodeFrame3();
    void initNodeFrame5();
    void initNodeFrame6();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
    rclcpp::TimerBase::SharedPtr serial_read_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // Watchdog state: last_frame_time_ is touched from each frame callback;
    // diagnosticsTimer compares it against the configured warn/error timeouts.
    rclcpp::Time last_frame_time_;
    double frame_timeout_warn_sec_{1.0};
    double frame_timeout_error_sec_{5.0};

    rclcpp::Publisher<anchorframe0>::SharedPtr pub_anchor_frame0_;
    rclcpp::Publisher<tagframe0>::SharedPtr pub_tag_frame0_;
    rclcpp::Publisher<nodeframe0>::SharedPtr pub_node_frame0_;
    rclcpp::Publisher<nodeframe1>::SharedPtr pub_node_frame1_;
    rclcpp::Publisher<nodeframe2>::SharedPtr pub_node_frame2_;
    rclcpp::Publisher<nodeframe3>::SharedPtr pub_node_frame3_;
    rclcpp::Publisher<nodeframe5>::SharedPtr pub_node_frame5_;
    rclcpp::Publisher<nodeframe6>::SharedPtr pub_node_frame6_;
  };
} // namespace linktrack

#endif // LINKTRACKINIT_H

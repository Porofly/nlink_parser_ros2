#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <thread>

#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe3.hpp>

#include "nlink_unpack/nlink_utils.h"
#include "protocol_extracter/nprotocol_extracter.h"
#include "protocol_manager.h"

using nodeframe3   = nlink_parser_ros2_interfaces::msg::LinktrackNodeframe3;

namespace linktrack
{
  class Init : public rclcpp::Node
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial);
    ~Init();

  private:
    NProtocolExtracter* protocol_extraction_;
    ProtocolManager protocol_manager_;
    anchorframe0 buffer_msg_anchorframe0_;
    tagframe0 buffer_msg_tagframe0_;
    nodeframe0 buffer_msg_nodeframe0_;
    nodeframe1 buffer_msg_nodeframe1_;
    nodeframe2 buffer_msg_nodeframe2_;
    nodeframe3 buffer_msg_nodeframe3_;
    nodeframe5 buffer_msg_nodeframe5_;
    nodeframe6 buffer_msg_nodeframe6_;

    void initDataTransmission();
    void serialReadTimer();
    void nodeFramePublisher();
    void initAnchorFrame0(NProtocolExtracter *protocol_extraction);
    void initTagFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame1(NProtocolExtracter *protocol_extraction);
    void initNodeFrame2(NProtocolExtracter *protocol_extraction);
    void initNodeFrame3(NProtocolExtracter *protocol_extraction);
    void startSerialReadThread();

    serial::Serial *serial_;
    NProtocolExtracter *protocol_extraction_;

    std::thread serial_thread_;

    rclcpp::Publisher<nodeframe3>  ::SharedPtr pub_node_frame3_;
  };
} // namespace linktrack

#endif // LINKTRACKINIT_H
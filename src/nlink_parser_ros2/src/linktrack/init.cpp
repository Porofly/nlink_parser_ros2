#include "init.h"
#include "nutils.h"
#include "protocols.h"
#include <thread>

#define ARRAY_ASSIGN(DEST, SRC)                                                \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT)           \
  {                                                                            \
    DEST[_CNT] = SRC[_CNT];                                                    \
  }

namespace linktrack
{
  nodeframe3   g_msg_nodeframe3;

  Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial)
    : Node("linktrack_ros2")
    , serial_thread_()
  {
    serial_ = serial;
    protocol_extraction_ = protocol_extraction;

    this->declare_parameter<int>("system_id", 1);
    int system_id = this->get_parameter("system_id").as_int();

    this->declare_parameter<int>("tag_id", 1);
    int tag_id = this->get_parameter("tag_id").as_int();

    initNodeFrame3(protocol_extraction);
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    std::string topic_name = "drone" + std::to_string(system_id) +"/tag" + std::to_string(tag_id) + "/nlink_linktrack_nodeframe3";
    pub_node_frame3_ = create_publisher<nodeframe3>(topic_name, qos);

    startSerialReadThread();

    RCLCPP_INFO(this->get_logger(), "Initialized linktrack");
  }

  Init::~Init()
  {
    if (serial_thread_.joinable()) {
      serial_thread_.join();
    }
  }

  void Init::startSerialReadThread()
  {
    serial_thread_ = std::thread([this]() {
      while (rclcpp::ok()) {
        try {
          // Blocks until data is readable or the serial read timeout
          // elapses, so frames are drained (and published) as soon as
          // they arrive instead of being batched on a fixed poll period.
          if (this->serial_->waitReadable()) {
            size_t available_bytes = this->serial_->available();
            if (available_bytes > 0) {
              // Must start empty: serial::Serial::read APPENDS to the string
              // (serial.cc:163), so sizing it first prepends that many NUL bytes
              // to every chunk. Harmless while a chunk held whole frames, but
              // waitReadable() returns one USB packet (64 B) at a time, so any
              // frame longer than that is split -- and the next chunk's NULs land
              // in the middle of it, failing the checksum. NodeFrame3 is
              // 22 + 7*nodes bytes, so it crosses 64 B at 6 anchors and every
              // frame is then dropped. linktrack_aoa/init.cpp already does this
              // right.
              std::string buf;
              this->serial_->read(buf, available_bytes);

              protocol_extraction_->AddNewData(buf);
            }
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
        }
      }
    });
  }

  void Init::initNodeFrame3(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolNodeFrame3;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {

      const auto &data = g_nlt_nodeframe3.result;
      auto &msg_data = g_msg_nodeframe3;
      auto &msg_nodes = msg_data.nodes;

      msg_data.role = data.role;
      msg_data.id = data.id;
      msg_data.local_time = data.local_time;
      msg_data.system_time = data.system_time;
      msg_data.voltage = data.voltage;

      msg_nodes.resize(data.valid_node_count);
      for (size_t i = 0; i < data.valid_node_count; ++i)
      {
        auto &msg_node = msg_nodes[i];
        auto node = data.nodes[i];
        msg_node.id = node->id;
        msg_node.role = node->role;
        msg_node.dis = node->dis;
        msg_node.fp_rssi = node->fp_rssi;
        msg_node.rx_rssi = node->rx_rssi;
      }
      pub_node_frame3_->publish(msg_data);
    });
  }

} // namespace linktrack

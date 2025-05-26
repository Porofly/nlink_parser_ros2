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
  nodeframe2   g_msg_nodeframe2;

  Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial)
    : Node("linktrack_ros2")
    , serial_thread_()
  {
    serial_ = serial;
    protocol_extraction_ = protocol_extraction;

    initNodeFrame2(protocol_extraction);
    rclcpp::QoS qos(rclcpp::KeepLast(200));
    pub_node_frame2_   = create_publisher<nodeframe2>(  "nlink_linktrack_nodeframe2",   qos);

    // Start serial read thread
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
    // define a delay for serial read
    std::chrono::milliseconds delay_ms(40);

    serial_thread_ = std::thread([this, delay_ms]() {
      std::string buf;
      while (rclcpp::ok()) {
        try {
          size_t available_bytes = this->serial_->available();
          if (available_bytes > 0) {
            std::string buf;
            buf.resize(available_bytes);
            this->serial_->read(buf, available_bytes);

            protocol_extraction_->AddNewData(buf);
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
        }
        std::this_thread::sleep_for(delay_ms);
      }
    });
  }

  void Init::initNodeFrame2(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolNodeFrame2;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {

      const auto &data = g_nlt_nodeframe2.result;
      auto &msg_data = g_msg_nodeframe2;
      auto &msg_nodes = msg_data.nodes;

      msg_data.role = data.role;
      msg_data.id = data.id;
      msg_data.local_time = data.local_time;
      msg_data.system_time = data.system_time;
      msg_data.voltage = data.voltage;
      ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
      ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
      ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
      ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
      ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
      ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
      ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

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
      pub_node_frame2_->publish(msg_data);
    });
  }

} // namespace linktrack
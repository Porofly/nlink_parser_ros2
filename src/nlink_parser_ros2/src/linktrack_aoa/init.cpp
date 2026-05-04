#include "init.h"

#include "init_serial.h"
#include "nlink_unpack/nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe0.h"
#include "nutils.h"

class NLTAoa_ProtocolNodeFrame0 : public NLinkProtocolVLength
{
public:
  NLTAoa_ProtocolNodeFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

NLTAoa_ProtocolNodeFrame0::NLTAoa_ProtocolNodeFrame0()
    : NLinkProtocolVLength(
          true, g_nltaoa_nodeframe0.fixed_part_size,
          {g_nltaoa_nodeframe0.frame_header, g_nltaoa_nodeframe0.function_mark})
{
}

void NLTAoa_ProtocolNodeFrame0::UnpackFrameData(const uint8_t *data)
{
  g_nltaoa_nodeframe0.UnpackData(data, length());
}

namespace {
template <typename Arr>
size_t clamp_node_count(size_t reported, const Arr& arr) {
  constexpr size_t kCapacity = sizeof(arr) / sizeof(arr[0]);
  return reported < kCapacity ? reported : kCapacity;
}
}  // namespace

namespace linktrack_aoa
{
  Init::Init() : Node("linktrack_aoa_ros2")
  {
    this->declare_parameter<std::string>("port_name", "/dev/ttyCH343USB0");
    this->declare_parameter<int>("baudrate", 921600);
    this->declare_parameter<std::string>("frame_id", "uwb_link");
    this->declare_parameter<double>("serial_read_rate_hz", 100.0);

    const auto port_name = this->get_parameter("port_name").as_string();
    const auto baudrate = static_cast<uint32_t>(this->get_parameter("baudrate").as_int());
    frame_id_ = this->get_parameter("frame_id").as_string();
    const double read_rate_hz = this->get_parameter("serial_read_rate_hz").as_double();
    const auto read_period = std::chrono::microseconds(
        static_cast<int64_t>(1.0e6 / std::max(1.0, read_rate_hz)));

    if (!initSerial(serial_, port_name, baudrate, this->get_logger())) {
      return;
    }

    protocol_extraction_ = std::make_unique<NProtocolExtracter>();

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    pub_node_frame0_ = create_publisher<nlink_parser_ros2_interfaces::msg::LinktrackNodeframe0>(
        "nlink_linktrack_nodeframe0", qos);
    pub_aoa_node_frame0_ = create_publisher<nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0>(
        "nlink_linktrack_aoa_nodeframe0", qos);

    initDataTransmission();
    initNodeFrame0();
    InitAoaNodeFrame0();

    serial_read_timer_ = this->create_wall_timer(
        read_period, std::bind(&Init::serialReadTimer, this));

    RCLCPP_INFO(this->get_logger(),
                "linktrack_aoa initialized: port=%s @ %u baud, frame_id='%s', serial_read_rate=%.1f Hz",
                port_name.c_str(), baudrate, frame_id_.c_str(), read_rate_hz);
  }

  void Init::serialReadTimer(){
    if (!this->serial_.isOpen()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Serial port is not open");
      return;
    }
    try {
      auto available_bytes = this->serial_.available();
      if (available_bytes) {
        std::string str_received;
        this->serial_.read(str_received, available_bytes);
        this->protocol_extraction_->AddNewData(str_received);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Serial read error: %s", e.what());
    }
  }

  void Init::initDataTransmission()
  {
    auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
      if (this->serial_.isOpen()) {
        try {
          this->serial_.write(msg->data);
        } catch (const std::exception& e) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Serial write error: %s", e.what());
        }
      }
    };
    dt_sub_ =
        create_subscription<std_msgs::msg::String>("nlink_linktrack_data_transmission", 10, callback);
  }

  void Init::initNodeFrame0()
  {
    auto protocol = protocol_manager_.addProtocol<NLT_ProtocolNodeFrame0>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = g_nlt_nodeframe0.result;
      nlink_parser_ros2_interfaces::msg::LinktrackNodeframe0 msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      msg.role = data.role;
      msg.id = data.id;
      const size_t icount = clamp_node_count(data.valid_node_count, data.nodes);
      msg.nodes.resize(icount);
      for (size_t i = 0; i < icount; ++i)
      {
        auto &msg_node = msg.nodes[i];
        auto node = data.nodes[i];
        msg_node.id = node->id;
        msg_node.role = node->role;
        msg_node.data.resize(node->data_length);
        memcpy(msg_node.data.data(), node->data, node->data_length);
      }
      pub_node_frame0_->publish(msg);
    });
  }

  void Init::InitAoaNodeFrame0()
  {
    auto protocol = protocol_manager_.addProtocol<NLTAoa_ProtocolNodeFrame0>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = g_nltaoa_nodeframe0.result;
      nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0 msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      msg.role = data.role;
      msg.id = data.id;
      msg.local_time = data.local_time;
      msg.system_time = data.system_time;
      msg.voltage = data.voltage;
      const size_t icount = clamp_node_count(data.valid_node_count, data.nodes);
      msg.nodes.resize(icount);
      for (size_t i = 0; i < icount; ++i)
      {
        auto &msg_node = msg.nodes[i];
        auto node = data.nodes[i];
        msg_node.id = node->id;
        msg_node.role = node->role;
        msg_node.dis = node->dis;
        msg_node.angle = node->angle;
        msg_node.fp_rssi = node->fp_rssi;
        msg_node.rx_rssi = node->rx_rssi;
      }
      pub_aoa_node_frame0_->publish(msg);
    });
  }

} // namespace linktrack_aoa

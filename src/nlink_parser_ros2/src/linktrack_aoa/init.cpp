#include "init.h"

#include "defaults.h"
#include "init_serial.h"
#include "nlink_unpack/nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe0.h"

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
  namespace defaults_ = nlink_parser_ros2::defaults;

  Init::Init() : Node("linktrack_aoa_ros2"), last_reconnect_attempt_(0, 0, RCL_ROS_TIME)
  {
    this->declare_parameter<std::string>("port_name", defaults_::kPortName);
    this->declare_parameter<int>("baudrate", defaults_::kBaudrate);
    this->declare_parameter<std::string>("frame_id", defaults_::kFrameId);
    this->declare_parameter<double>("serial_read_rate_hz", defaults_::kSerialReadRateHz);

    port_name_ = this->get_parameter("port_name").as_string();
    baudrate_ = static_cast<uint32_t>(this->get_parameter("baudrate").as_int());
    frame_id_ = this->get_parameter("frame_id").as_string();
    const double read_rate_hz = this->get_parameter("serial_read_rate_hz").as_double();
    const auto read_period = std::chrono::microseconds(
        static_cast<int64_t>(1.0e6 / std::max(1.0, read_rate_hz)));

    if (!tryOpenSerial()) {
      RCLCPP_WARN(this->get_logger(),
                  "Initial serial open failed; will keep retrying every %lds",
                  static_cast<long>(defaults_::kReconnectInterval.count()));
    }

    protocol_extraction_ = std::make_unique<NProtocolExtracter>();

    rclcpp::QoS qos(rclcpp::KeepLast(defaults_::kPublishQueueDepth));
    pub_node_frame0_ = create_publisher<nlink_parser_ros2_interfaces::msg::LinktrackNodeframe0>(
        "nlink_linktrack_nodeframe0", qos);
    pub_aoa_node_frame0_ = create_publisher<nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0>(
        "nlink_linktrack_aoa_nodeframe0", qos);

    initDataTransmission();
    initNodeFrame0();
    InitAoaNodeFrame0();

    serial_read_timer_ = this->create_wall_timer(
        read_period, std::bind(&Init::serialReadTimer, this));

    initialized_ = true;
    RCLCPP_INFO(this->get_logger(),
                "linktrack_aoa initialized: port=%s @ %u baud, frame_id='%s', serial_read_rate=%.1f Hz",
                port_name_.c_str(), baudrate_, frame_id_.c_str(), read_rate_hz);
  }

  bool Init::tryOpenSerial() {
    if (serial_.isOpen()) {
      return true;
    }
    return initSerial(serial_, port_name_, baudrate_, this->get_logger());
  }

  void Init::serialReadTimer() {
    if (!this->serial_.isOpen()) {
      const auto now = this->now();
      if ((now - last_reconnect_attempt_) <
          rclcpp::Duration(defaults_::kReconnectInterval)) {
        return;
      }
      last_reconnect_attempt_ = now;
      if (tryOpenSerial()) {
        RCLCPP_INFO(this->get_logger(), "Serial port reconnected");
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                             defaults_::kErrorThrottleMs,
                             "Serial port still unavailable (%s)",
                             port_name_.c_str());
      }
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
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),
                            defaults_::kErrorThrottleMs,
                            "Serial read error: %s; closing port for reconnect",
                            e.what());
      try { serial_.close(); } catch (...) { /* ignore */ }
    }
  }

  void Init::initDataTransmission()
  {
    auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
      if (this->serial_.isOpen()) {
        try {
          this->serial_.write(msg->data);
        } catch (const std::exception& e) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),
                                defaults_::kErrorThrottleMs,
                                "Serial write error: %s", e.what());
        }
      }
    };
    dt_sub_ = create_subscription<std_msgs::msg::String>(
        "nlink_linktrack_data_transmission",
        rclcpp::QoS(rclcpp::KeepLast(defaults_::kSubscriptionQueueDepth)),
        callback);
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

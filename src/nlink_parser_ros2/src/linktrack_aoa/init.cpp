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

  Init::Init()
      : Node("linktrack_aoa_ros2"),
        last_reconnect_attempt_(0, 0, RCL_ROS_TIME),
        last_frame_time_(0, 0, RCL_ROS_TIME)
  {
    this->declare_parameter<std::string>("port_name", defaults_::kPortName);
    this->declare_parameter<int>("baudrate", defaults_::kBaudrate);
    this->declare_parameter<std::string>("frame_id", defaults_::kFrameId);
    this->declare_parameter<double>("serial_read_rate_hz", defaults_::kSerialReadRateHz);
    this->declare_parameter<double>("frame_timeout_warn_sec", defaults_::kFrameTimeoutWarnSec);
    this->declare_parameter<double>("frame_timeout_error_sec", defaults_::kFrameTimeoutErrorSec);
    this->declare_parameter<double>("diagnostics_rate_hz", defaults_::kDiagnosticsRateHz);

    port_name_ = this->get_parameter("port_name").as_string();
    baudrate_ = static_cast<uint32_t>(this->get_parameter("baudrate").as_int());
    frame_id_ = this->get_parameter("frame_id").as_string();
    const double read_rate_hz = this->get_parameter("serial_read_rate_hz").as_double();
    frame_timeout_warn_sec_ = this->get_parameter("frame_timeout_warn_sec").as_double();
    frame_timeout_error_sec_ = this->get_parameter("frame_timeout_error_sec").as_double();
    const double diag_rate_hz = this->get_parameter("diagnostics_rate_hz").as_double();
    const auto read_period = std::chrono::microseconds(
        static_cast<int64_t>(1.0e6 / std::max(1.0, read_rate_hz)));
    const auto diag_period = std::chrono::microseconds(
        static_cast<int64_t>(1.0e6 / std::max(0.1, diag_rate_hz)));

    last_reconnect_attempt_ = this->now();
    last_frame_time_ = this->now();

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
    pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(defaults_::kPublishQueueDepth)));

    initDataTransmission();
    initNodeFrame0();
    InitAoaNodeFrame0();

    serial_read_timer_ = this->create_wall_timer(
        read_period, std::bind(&Init::serialReadTimer, this));
    diagnostics_timer_ = this->create_wall_timer(
        diag_period, std::bind(&Init::diagnosticsTimer, this));

    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          return onSetParameters(params);
        });

    initialized_ = true;
    RCLCPP_INFO(this->get_logger(),
                "linktrack_aoa initialized: port=%s @ %u baud, frame_id='%s', "
                "serial_read_rate=%.1f Hz, watchdog warn/err=%.1f/%.1f s",
                port_name_.c_str(), baudrate_, frame_id_.c_str(), read_rate_hz,
                frame_timeout_warn_sec_, frame_timeout_error_sec_);
  }

  rcl_interfaces::msg::SetParametersResult Init::onSetParameters(
      const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool serial_changed = false;
    for (const auto& p : params) {
      if (p.get_name() == "port_name" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        port_name_ = p.as_string();
        serial_changed = true;
      } else if (p.get_name() == "baudrate" &&
                 p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        baudrate_ = static_cast<uint32_t>(p.as_int());
        serial_changed = true;
      } else if (p.get_name() == "frame_id" &&
                 p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        frame_id_ = p.as_string();
      } else if (p.get_name() == "frame_timeout_warn_sec" &&
                 p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        frame_timeout_warn_sec_ = p.as_double();
      } else if (p.get_name() == "frame_timeout_error_sec" &&
                 p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        frame_timeout_error_sec_ = p.as_double();
      }
    }
    if (serial_changed) {
      RCLCPP_INFO(this->get_logger(),
                  "Serial config changed via parameter; closing for reconnect on %s @ %u",
                  port_name_.c_str(), baudrate_);
      try { serial_.close(); } catch (...) { /* ignore */ }
      last_reconnect_attempt_ = this->now() -
          rclcpp::Duration(defaults_::kReconnectInterval);
    }
    return result;
  }

  void Init::diagnosticsTimer() {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(this->get_name()) + ": liveness";
    status.hardware_id = port_name_;

    const double age = (this->now() - last_frame_time_).seconds();
    const bool port_open = serial_.isOpen();

    if (!port_open) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Serial port closed";
    } else if (age > frame_timeout_error_sec_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "No frames received for " + std::to_string(age) + "s";
    } else if (age > frame_timeout_warn_sec_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Frame age " + std::to_string(age) + "s exceeds warn threshold";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "OK";
    }

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "port_open";
    kv.value = port_open ? "true" : "false";
    status.values.push_back(kv);
    kv.key = "frame_age_sec";
    kv.value = std::to_string(age);
    status.values.push_back(kv);

    msg.status.push_back(status);
    pub_diagnostics_->publish(msg);
  }

  bool Init::tryOpenSerial(bool quiet) {
    if (serial_.isOpen()) {
      return true;
    }
    return initSerial(serial_, port_name_, baudrate_, this->get_logger(), quiet);
  }

  void Init::serialReadTimer() {
    if (!this->serial_.isOpen()) {
      const auto now = this->now();
      if ((now - last_reconnect_attempt_) <
          rclcpp::Duration(defaults_::kReconnectInterval)) {
        return;
      }
      last_reconnect_attempt_ = now;
      if (tryOpenSerial(/*quiet=*/true)) {
        RCLCPP_INFO(this->get_logger(), "Serial port reconnected on %s",
                    port_name_.c_str());
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
      noteFrameReceived();
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
      noteFrameReceived();
      pub_aoa_node_frame0_->publish(msg);
    });
  }

} // namespace linktrack_aoa

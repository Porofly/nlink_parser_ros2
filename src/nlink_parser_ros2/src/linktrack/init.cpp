#include "init.h"

#include "init_serial.h"
#include "nutils.h"
#include "protocols.h"

#define ARRAY_ASSIGN(DEST, SRC)                                                \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT)           \
  {                                                                            \
    DEST[_CNT] = SRC[_CNT];                                                    \
  }

namespace {
// Clamp the device-reported valid_node_count to the static capacity of the
// upstream parser's nodes[] array. The parser already validates incoming
// frames, but this is a defensive boundary at the ROS 2 wrapper layer:
// a malformed device payload must never cause an out-of-bounds read here.
template <typename Arr>
size_t clamp_node_count(size_t reported, const Arr& arr) {
  constexpr size_t kCapacity = sizeof(arr) / sizeof(arr[0]);
  return reported < kCapacity ? reported : kCapacity;
}
}  // namespace

namespace linktrack
{
  Init::Init() : Node("linktrack_ros2")
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
    pub_anchor_frame0_= create_publisher<anchorframe0>("nlink_linktrack_anchorframe0", qos);
    pub_tag_frame0_= create_publisher<tagframe0>("nlink_linktrack_tagframe0", qos);
    pub_node_frame0_= create_publisher<nodeframe0>("nlink_linktrack_nodeframe0", qos);
    pub_node_frame1_= create_publisher<nodeframe1>("nlink_linktrack_nodeframe1", qos);
    pub_node_frame2_= create_publisher<nodeframe2>("nlink_linktrack_nodeframe2", qos);
    pub_node_frame3_= create_publisher<nodeframe3>("nlink_linktrack_nodeframe3", qos);
    pub_node_frame5_= create_publisher<nodeframe5>("nlink_linktrack_nodeframe5", qos);
    pub_node_frame6_= create_publisher<nodeframe6>("nlink_linktrack_nodeframe6", qos);

    initDataTransmission();
    initAnchorFrame0();
    initTagFrame0();
    initNodeFrame0();
    initNodeFrame1();
    initNodeFrame2();
    initNodeFrame3();
    initNodeFrame5();
    initNodeFrame6();

    serial_read_timer_ = this->create_wall_timer(
        read_period, std::bind(&Init::serialReadTimer, this));

    RCLCPP_INFO(this->get_logger(),
                "linktrack initialized: port=%s @ %u baud, frame_id='%s', serial_read_rate=%.1f Hz",
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

  void Init::initAnchorFrame0()
  {
    auto protocol = protocol_manager_.addProtocol<NLT_ProtocolAnchorFrame0>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = nlt_anchorframe0_.result;
      anchorframe0 msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      msg.role = data.role;
      msg.id = data.id;
      msg.voltage = data.voltage;
      msg.local_time = data.local_time;
      msg.system_time = data.system_time;
      auto &msg_nodes = msg.nodes;
      decltype(msg.nodes)::value_type msg_node;
      const size_t icount = clamp_node_count(data.valid_node_count, data.nodes);
      for (size_t i = 0; i < icount; ++i)
      {
        auto node = data.nodes[i];
        msg_node.role = node->role;
        msg_node.id = node->id;
        ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
        ARRAY_ASSIGN(msg_node.dis_arr, node->dis_arr)
        msg_nodes.push_back(msg_node);
      }
      pub_anchor_frame0_->publish(msg);
    });
  }

  void Init::initTagFrame0()
  {
    auto protocol = protocol_manager_.addProtocol<NLT_ProtocolTagFrame0>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = g_nlt_tagframe0.result;
      tagframe0 msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      msg.role = data.role;
      msg.id = data.id;
      msg.local_time = data.local_time;
      msg.system_time = data.system_time;
      msg.voltage = data.voltage;
      ARRAY_ASSIGN(msg.pos_3d, data.pos_3d)
      ARRAY_ASSIGN(msg.eop_3d, data.eop_3d)
      ARRAY_ASSIGN(msg.vel_3d, data.vel_3d)
      ARRAY_ASSIGN(msg.dis_arr, data.dis_arr)
      ARRAY_ASSIGN(msg.imu_gyro_3d, data.imu_gyro_3d)
      ARRAY_ASSIGN(msg.imu_acc_3d, data.imu_acc_3d)
      ARRAY_ASSIGN(msg.angle_3d, data.angle_3d)
      ARRAY_ASSIGN(msg.quaternion, data.quaternion)
      pub_tag_frame0_->publish(msg);
    });
  }

  void Init::initNodeFrame0()
  {
    auto protocol = protocol_manager_.addProtocol<NLT_ProtocolNodeFrame0>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = g_nlt_nodeframe0.result;
      nodeframe0 msg;
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

  void Init::initNodeFrame1()
  {
    auto protocol = protocol_manager_.addProtocol<NLT_ProtocolNodeFrame1>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = g_nlt_nodeframe1.result;
      nodeframe1 msg;
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
        ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
      }
      pub_node_frame1_->publish(msg);
    });
  }

  void Init::initNodeFrame2()
  {
    auto protocol = protocol_manager_.addProtocol<NLT_ProtocolNodeFrame2>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = g_nlt_nodeframe2.result;
      nodeframe2 msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      msg.role = data.role;
      msg.id = data.id;
      msg.local_time = data.local_time;
      msg.system_time = data.system_time;
      msg.voltage = data.voltage;
      ARRAY_ASSIGN(msg.pos_3d, data.pos_3d)
      ARRAY_ASSIGN(msg.eop_3d, data.eop_3d)
      ARRAY_ASSIGN(msg.vel_3d, data.vel_3d)
      ARRAY_ASSIGN(msg.imu_gyro_3d, data.imu_gyro_3d)
      ARRAY_ASSIGN(msg.imu_acc_3d, data.imu_acc_3d)
      ARRAY_ASSIGN(msg.angle_3d, data.angle_3d)
      ARRAY_ASSIGN(msg.quaternion, data.quaternion)
      const size_t icount = clamp_node_count(data.valid_node_count, data.nodes);
      msg.nodes.resize(icount);
      for (size_t i = 0; i < icount; ++i)
      {
        auto &msg_node = msg.nodes[i];
        auto node = data.nodes[i];
        msg_node.id = node->id;
        msg_node.role = node->role;
        msg_node.dis = node->dis;
        msg_node.fp_rssi = node->fp_rssi;
        msg_node.rx_rssi = node->rx_rssi;
      }
      pub_node_frame2_->publish(msg);
    });
  }

  void Init::initNodeFrame3()
  {
    auto protocol = protocol_manager_.addProtocol<NLT_ProtocolNodeFrame3>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = g_nlt_nodeframe3.result;
      nodeframe3 msg;
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
        msg_node.fp_rssi = node->fp_rssi;
        msg_node.rx_rssi = node->rx_rssi;
      }
      pub_node_frame3_->publish(msg);
    });
  }

  void Init::initNodeFrame5()
  {
    auto protocol = protocol_manager_.addProtocol<NLT_ProtocolNodeFrame5>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = g_nlt_nodeframe5.result;
      nodeframe5 msg;
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
        msg_node.fp_rssi = node->fp_rssi;
        msg_node.rx_rssi = node->rx_rssi;
      }
      pub_node_frame5_->publish(msg);
    });
  }

  void Init::initNodeFrame6()
  {
    auto protocol = protocol_manager_.addProtocol<NLT_ProtocolNodeFrame6>(protocol_extraction_.get());
    protocol->SetHandleDataCallback([this]() {
      const auto &data = g_nlt_nodeframe6.result;
      nodeframe6 msg;
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
      pub_node_frame6_->publish(msg);
    });
  }

} // namespace linktrack

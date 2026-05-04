#ifndef NLINK_PARSER_ROS2_DEFAULTS_H
#define NLINK_PARSER_ROS2_DEFAULTS_H

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace nlink_parser_ros2::defaults {

// Serial port defaults. The CH343 USB-UART module enumerates as
// /dev/ttyCH343USBx when the WCH driver is loaded; users on stock
// Ubuntu kernels may want to override this to /dev/ttyACM* or
// /dev/ttyUSB*.
constexpr const char* kPortName = "/dev/ttyCH343USB0";
constexpr int kBaudrate = 921600;

// ROS 2 frame id used for every published message header. Override
// per-instance via the `frame_id` parameter.
constexpr const char* kFrameId = "uwb_link";

// Rate at which the node drains the serial RX buffer. 100 Hz keeps
// the OS-level buffer well below overflow at 921600 baud while
// avoiding excessive context switches.
constexpr double kSerialReadRateHz = 100.0;

// Default reconnection retry interval when the serial port closes
// unexpectedly (e.g. USB disconnect on a mobile platform).
constexpr auto kReconnectInterval = std::chrono::seconds(2);

// QoS depth for sensor data publishers and the data-transmission
// subscription. KeepLast(10) is the conventional choice for sensor
// topics in ROS 2.
constexpr size_t kPublishQueueDepth = 10;
constexpr size_t kSubscriptionQueueDepth = 10;

// Throttle period (ms) for repeated error logs from the serial path.
constexpr int64_t kErrorThrottleMs = 2000;

// Liveness watchdog. The node tracks the time of the most recent parsed
// frame and publishes diagnostic_msgs/DiagnosticArray on /diagnostics at
// kDiagnosticsRateHz. If the gap since the last frame exceeds the WARN
// threshold the level transitions OK -> WARN; past the ERROR threshold
// it becomes ERROR. Tune via parameters at launch.
constexpr double kDiagnosticsRateHz = 1.0;
constexpr double kFrameTimeoutWarnSec = 1.0;
constexpr double kFrameTimeoutErrorSec = 5.0;

}  // namespace nlink_parser_ros2::defaults

#endif  // NLINK_PARSER_ROS2_DEFAULTS_H

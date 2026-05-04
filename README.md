# nlink_parser_ros2

ROS 2 driver for [Nooploop LinkTrack / LinkTrack AoA](https://www.nooploop.com/) UWB modules.
Originally ported from the manufacturer's ROS 1 [nlink_parser](https://github.com/nooploop-dev/nlink_parser);
the port has since been refactored to follow ROS 2 idioms (parameters, lifecycle, headers, event-driven publishing).

> 한국어 문서: [README.ko.md](README.ko.md)

## Status

- **Supported devices**: LinkTrack P-A series (LP / DR / DT modes), LinkTrack AoA.
- **Not supported**: ToFSense (the upstream ROS 1 code has been removed; rewrite welcome).
- **Tested on**: ROS 2 Jazzy / Ubuntu 24.04 / kernel 6.x.

## Workspace layout

```
nlink_parser_ros2/
├── src/
│   ├── nlink_parser_ros2/             # Main node + launch + params
│   ├── nlink_parser_ros2_interfaces/  # ROS 2 message definitions
│   └── serial/                         # Submodule: Sunnybotics/serial (ROS 2 branch)
└── README.md / README.ko.md
```

The third-party C parsers (`nlink_unpack`, `protocol_extracter`) are vendored under
`src/nlink_parser_ros2/src/utils/` and are not git submodules.

## Prerequisites

### ROS 2 Jazzy

Install per the [official guide](https://docs.ros.org/en/jazzy/Installation.html), then
`source /opt/ros/jazzy/setup.bash` in every shell that builds or launches this workspace.

### Serial driver for the CH343 USB-UART

The LinkTrack P-A USB cable enumerates as a WCH CH343 device. Two paths:

1. **Stock kernel (cdc_acm)** — many Ubuntu 24.04 setups already expose the device as
   `/dev/ttyACM*`. Try this first; if it works, skip step 2.
2. **WCH ch343 driver** — required if cdc_acm does not enumerate the port. Build from
   [WCHSoftGroup/ch343ser_linux](https://github.com/WCHSoftGroup/ch343ser_linux):
   ```bash
   cd ch343ser_linux/driver
   make            # builds ch343.ko
   sudo make load  # one-shot insmod
   sudo make install  # persist across reboots
   ```
   On kernel 6.x the source includes `<asm/unaligned.h>` which has moved to
   `<linux/unaligned.h>`; patch line 61 of `ch343.c` accordingly before `make`.

After either path the device should appear as `/dev/ttyCH343USB0` (WCH) or `/dev/ttyACM0` (cdc_acm).

### User must be in the `dialout` group

```bash
sudo usermod -aG dialout "$USER"
# log out and back in, or `newgrp dialout`
```

## Build

```bash
git clone --recursive <this-repo>
cd nlink_parser_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

> Submodules are pinned via `.gitmodules` (`update = checkout`). Use
> `git submodule update --init`, **not** `--remote`, to keep reproducible builds.

## Launch

```bash
source install/setup.bash
ros2 launch nlink_parser_ros2 linktrack.launch.py
# or
ros2 launch nlink_parser_ros2 linktrack_aoa.launch.py
```

Override parameters from the command line:

```bash
ros2 launch nlink_parser_ros2 linktrack.launch.py \
    --ros-args -p port_name:=/dev/ttyACM0 -p frame_id:=base_link
```

## Parameters

| Name                  | Type   | Default                | Description                                |
|-----------------------|--------|------------------------|--------------------------------------------|
| `port_name`           | string | `/dev/ttyCH343USB0`    | Serial device path                         |
| `baudrate`            | int    | `921600`               | UART baud rate                             |
| `frame_id`            | string | `uwb_link`             | `header.frame_id` for every published msg  |
| `serial_read_rate_hz` | double | `100.0`                | Rate at which the RX buffer is drained     |

If the serial port disconnects mid-run (e.g. USB cable unplugged on a mobile platform),
the node will automatically attempt reconnection every 2 seconds and log when the link returns.

## Topic / mode mapping (LinkTrack)

LinkTrack publishes a different protocol frame depending on the configured Mode/Role
(set via Nooploop's NAssistant tool). Frames the active mode does not produce will not
appear on their topic — that is the intended behaviour.

| Mode / role          | Active topics                                             |
|----------------------|-----------------------------------------------------------|
| LP — TAG             | `tagframe0`, `nodeframe2`, `nodeframe3`                   |
| LP — ANCHOR/CONSOLE  | `anchorframe0`, `nodeframe1`, `nodeframe0` (data RX)      |
| DR_MODE0             | `nodeframe2`, `nodeframe3`, `nodeframe0` (data RX)        |
| DR_MODE1             | `nodeframe5`, `nodeframe6`                                |

`/nlink_linktrack_data_transmission` is a `std_msgs/String` **subscription**: publish to
it from your own node to broadcast a UWB payload to every peer in range.

In DR mode the `pos_3d / vel_3d / quaternion / imu_*` fields of `nodeframe2` are zero —
the hardware does not solve a position there. See [datasheet](docs) §5.1.2 for the full
field validity table; the relevant constraints are also noted in each `.msg` file.

## License

BSD 3-Clause. See [src/nlink_parser_ros2/LICENSE](src/nlink_parser_ros2/LICENSE).
The vendored submodules retain their upstream licenses.

## Maintainer

YG_Kim — `yeogyeom1@chungbuk.ac.kr`

Original ROS 2 port by Aarush Gupta (HopeTechnik); upstream ROS 1 code by Samuel Hsu (Nooploop).

## Contributing

Bugs / feature requests: open an issue on this repository.
For style: this project ships `.clang-format` (Google, 100 col) and `.clang-tidy`;
please run them on your changes before submitting a PR.

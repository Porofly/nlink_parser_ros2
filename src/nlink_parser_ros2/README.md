# nlink_parser_ros2 (package)

ROS 2 driver node for Nooploop LinkTrack and LinkTrack AoA UWB modules.

For workspace-level setup (build, drivers, hardware) see the
[top-level README](../../README.md).

## Executables

| Target                 | Source                            | Purpose                                            |
|------------------------|-----------------------------------|----------------------------------------------------|
| `linktrack`            | `src/linktrack/`                  | LinkTrack P-A series — LP / DR / DT modes          |
| `linktrack_aoa`        | `src/linktrack_aoa/`              | LinkTrack AoA — distance + angle of arrival        |
| `nlink_viz_ros2`       | `src/aoa_target.cpp`              | Convert AoA frame to `geometry_msgs/PoseStamped`   |
| `nlink_viz_single_ros2`| `src/target_single.cpp`           | Single-target variant of the above                 |

## Launch files

| Launch                       | What it starts                          |
|------------------------------|-----------------------------------------|
| `linktrack.launch.py`        | `linktrack` node with default params    |
| `linktrack_aoa.launch.py`    | `linktrack_aoa` node with default params|

Both launches load `params/linktrack_init_params.yaml` (or the AoA variant) using the
ROS 2 standard `parameters=[file]` mechanism, so command-line overrides like
`--ros-args -p port_name:=/dev/ttyACM0` work out of the box.

## Parameters

See the parameter table in the top-level README. They are declared on the node and
queryable via `ros2 param get/set`.

## Topics — published

All output messages now carry a populated `std_msgs/Header` (`stamp` from the node
clock, `frame_id` from the parameter).

| Topic                                | Type                                  | When it carries data         |
|--------------------------------------|---------------------------------------|------------------------------|
| `/nlink_linktrack_anchorframe0`      | `LinktrackAnchorframe0`               | LP mode — Anchor / Console   |
| `/nlink_linktrack_tagframe0`         | `LinktrackTagframe0`                  | LP mode — Tag                |
| `/nlink_linktrack_nodeframe0`        | `LinktrackNodeframe0`                 | UWB data RX (any mode)       |
| `/nlink_linktrack_nodeframe1`        | `LinktrackNodeframe1`                 | LP mode — Anchor / Console   |
| `/nlink_linktrack_nodeframe2`        | `LinktrackNodeframe2`                 | LP — Tag, or DR_MODE0        |
| `/nlink_linktrack_nodeframe3`        | `LinktrackNodeframe3`                 | LP — Tag, or DR_MODE0        |
| `/nlink_linktrack_nodeframe5`        | `LinktrackNodeframe5`                 | DR_MODE1                     |
| `/nlink_linktrack_nodeframe6`        | `LinktrackNodeframe6`                 | DR_MODE1                     |
| `/nlink_linktrack_aoa_nodeframe0`    | `LinktrackAoaNodeframe0`              | LinkTrack AoA only           |

Topics whose mode is not active never publish — there is no longer a 2 s timer that
re-broadcasts buffered defaults.

## Topics — subscribed

* `/nlink_linktrack_data_transmission` (`std_msgs/String`) — payload is forwarded
  verbatim over UWB to peer nodes. The peer receives it on its
  `/nlink_linktrack_nodeframe0`.

## Architecture notes

* The node owns the `serial::Serial` instance directly (no raw pointers passed in).
* Every device frame produces an immediate publish from the parser callback —
  there is no internal buffering or timer-driven re-publication.
* Serial is drained by a wall-timer at `serial_read_rate_hz`; if the port is closed
  the same timer attempts reconnection every 2 s.
* `valid_node_count` from device frames is clamped against the upstream parser's
  fixed-size `nodes[]` capacity before indexing — defensive bound at the wrapper.

## License

BSD 3-Clause — see [LICENSE](LICENSE).

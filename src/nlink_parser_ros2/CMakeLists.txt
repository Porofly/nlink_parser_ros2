cmake_minimum_required(VERSION 3.5.0)
project(nlink_parser_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlink_parser_ros2_interfaces REQUIRED)
find_package(serial REQUIRED)

include(GNUInstallDirs)

include_directories(include
    src/utils
    ${serial_INCLUDE_DIRS}
)

set(nutils "nutils")
add_library(${nutils}
    src/utils/protocol_extracter/nprotocol_base.cpp
    src/utils/protocol_extracter/nprotocol_extracter.cpp
    src/utils/nlink_unpack/nlink_utils.c
    src/utils/nlink_unpack/nlink_linktrack_anchorframe0.c
    src/utils/nlink_unpack/nlink_linktrack_tagframe0.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe0.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe1.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe2.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe3.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe5.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe6.c
    src/utils/nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
    src/utils/init_serial.cpp
    src/utils/nlink_protocol.cpp
)
target_link_libraries(${nutils}
  serial::serial
  rclcpp::rclcpp
)
ament_target_dependencies(${nutils}
  "rclcpp"
  "nlink_parser_ros2_interfaces"
  "std_msgs"
  "geometry_msgs"
)

set(LINKTRACK_NAME "linktrack")
add_executable(${LINKTRACK_NAME}
    src/linktrack/init.cpp
    src/linktrack/main.cpp
    src/linktrack/protocols.cpp
)
target_link_libraries(${LINKTRACK_NAME} ${nutils})
ament_target_dependencies(${LINKTRACK_NAME}
  "rclcpp"
  "nlink_parser_ros2_interfaces"
  "std_msgs"
)

set(LINKTRACK_AOA_NAME "linktrack_aoa")
add_executable(${LINKTRACK_AOA_NAME}
    src/linktrack_aoa/init.cpp
    src/linktrack_aoa/main.cpp
    src/linktrack/protocols.cpp
)
target_link_libraries(${LINKTRACK_AOA_NAME} ${nutils})
ament_target_dependencies(${LINKTRACK_AOA_NAME}
  "rclcpp"
  "nlink_parser_ros2_interfaces"
  "std_msgs"
  "geometry_msgs"
)

add_executable(nlink_viz_ros2 src/aoa_target.cpp)
target_link_libraries(nlink_viz_ros2 ${nutils})
ament_target_dependencies(nlink_viz_ros2
  "rclcpp"
  "nlink_parser_ros2_interfaces"
  "std_msgs"
  "geometry_msgs"
)

add_executable(nlink_viz_single_ros2 src/target_single.cpp)
target_link_libraries(nlink_viz_single_ros2 ${nutils})
ament_target_dependencies(nlink_viz_single_ros2
  "rclcpp"
  "nlink_parser_ros2_interfaces"
  "std_msgs"
  "geometry_msgs"
)

install(TARGETS
  ${LINKTRACK_NAME}
  ${LINKTRACK_AOA_NAME}
  nlink_viz_ros2
  nlink_viz_single_ros2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

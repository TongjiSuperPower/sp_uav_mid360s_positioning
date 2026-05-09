cmake_minimum_required(VERSION 3.8)
project(px4_sim_flight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(sensor_fusion_bridge src/sensor_fusion_bridge.cpp)
ament_target_dependencies(sensor_fusion_bridge rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(px4_odom_bridge src/px4_odom_bridge.cpp)
ament_target_dependencies(px4_odom_bridge rclcpp nav_msgs geometry_msgs px4_msgs tf2 tf2_geometry_msgs)

add_executable(offboard_landing_node src/offboard_landing_node.cpp)
ament_target_dependencies(offboard_landing_node rclcpp geometry_msgs px4_msgs)

# --- 手动 Gazebo 桥接 ---
find_library(GZ_TRANSPORT_LIB gz-transport13 HINTS /usr/lib/x86_64-linux-gnu)
find_library(GZ_MSGS_LIB gz-msgs10 HINTS /usr/lib/x86_64-linux-gnu)
if(NOT GZ_TRANSPORT_LIB)
  find_library(GZ_TRANSPORT_LIB gz-transport12 REQUIRED)
endif()
if(NOT GZ_MSGS_LIB)
  find_library(GZ_MSGS_LIB gz-msgs9 REQUIRED)
endif()

# 显式查找 protobuf（gz-msgs 的依赖）
find_library(PROTOBUF_LIB protobuf HINTS /usr/lib/x86_64-linux-gnu REQUIRED)

add_executable(manual_gz_bridge src/manual_gz_bridge.cpp)
ament_target_dependencies(manual_gz_bridge rclcpp sensor_msgs geometry_msgs rosgraph_msgs)
target_link_libraries(manual_gz_bridge ${GZ_TRANSPORT_LIB} ${GZ_MSGS_LIB} ${PROTOBUF_LIB})
target_include_directories(manual_gz_bridge PUBLIC
  /usr/include/gz/transport13
  /usr/include/gz/msgs10
  /usr/include/gz/utils2
)

install(TARGETS
  sensor_fusion_bridge
  px4_odom_bridge
  offboard_landing_node
  manual_gz_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config models worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

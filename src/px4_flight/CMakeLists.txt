cmake_minimum_required(VERSION 3.8)
project(px4_flight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExternalCommand.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 库文件
add_library(px4_flight_lib SHARED
  src/px4_interface.cpp
  src/external_pose_bridge.cpp
  src/flight_mode_manager.cpp
  src/offboard_controller.cpp
  src/command_mux.cpp
)

ament_target_dependencies(px4_flight_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  px4_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(px4_flight_lib "${cpp_typesupport_target}")

# 主节点
add_executable(main_flight_node src/main_flight_node.cpp)
target_link_libraries(main_flight_node px4_flight_lib)
ament_target_dependencies(main_flight_node rclcpp)

# Livox转换节点
add_executable(livox_converter src/livox_converter.cpp)
ament_target_dependencies(livox_converter rclcpp sensor_msgs)

# 安装
install(TARGETS
  px4_flight_lib
  main_flight_node
  livox_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME}/)

ament_package()

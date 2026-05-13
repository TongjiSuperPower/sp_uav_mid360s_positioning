cmake_minimum_required(VERSION 3.8)
project(px4_real_flight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)       # ← 新增（已有则保留）
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(odometry_to_vision_pose src/odometry_to_vision_pose.cpp)
ament_target_dependencies(odometry_to_vision_pose rclcpp nav_msgs geometry_msgs mavros_msgs)

add_executable(flight_monitor src/flight_monitor.cpp)
ament_target_dependencies(flight_monitor
    rclcpp
    geometry_msgs
    sensor_msgs
    mavros_msgs
)

add_executable(ned_position_listener src/ned_position_listener.cpp)
ament_target_dependencies(ned_position_listener
  rclcpp nav_msgs geometry_msgs tf2_ros tf2  # ← nav_msgs 已加入
)

add_executable(offboard_landing_test src/offboard_landing_test.cpp)
ament_target_dependencies(offboard_landing_test
    rclcpp
    geometry_msgs
    mavros_msgs
)

install(TARGETS
    odometry_to_vision_pose 
    flight_monitor
    ned_position_listener
    offboard_landing_test
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
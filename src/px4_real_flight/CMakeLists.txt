cmake_minimum_required(VERSION 3.8)
project(px4_real_flight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 

add_executable(odometry_to_vision_pose src/odometry_to_vision_pose.cpp)
ament_target_dependencies(odometry_to_vision_pose rclcpp nav_msgs geometry_msgs mavros_msgs)

add_executable(flight_monitor src/flight_monitor.cpp)
ament_target_dependencies(flight_monitor
    rclcpp
    geometry_msgs
    sensor_msgs
    mavros_msgs
)

install(TARGETS
    odometry_to_vision_pose 
    flight_monitor
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(qr_landing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(qr_detector_node src/qr_detector_node.cpp)
ament_target_dependencies(qr_detector_node
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(qr_detector_node ${OpenCV_LIBS})

add_executable(coordinate_transformer_node src/coordinate_transformer_node.cpp)
ament_target_dependencies(coordinate_transformer_node
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

add_executable(landing_controller_node src/landing_controller_node.cpp)
ament_target_dependencies(landing_controller_node
  rclcpp geometry_msgs std_msgs tf2
)

add_executable(qr_test_visualizer_node src/qr_test_visualizer_node.cpp)
ament_target_dependencies(qr_test_visualizer_node
  rclcpp sensor_msgs geometry_msgs cv_bridge tf2
)
target_link_libraries(qr_test_visualizer_node ${OpenCV_LIBS})

install(TARGETS
  qr_detector_node
  coordinate_transformer_node
  landing_controller_node
  qr_test_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
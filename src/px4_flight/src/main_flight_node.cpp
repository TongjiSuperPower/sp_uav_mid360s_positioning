#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "px4_flight/px4_interface.hpp"
#include "px4_flight/external_pose_bridge.hpp"
#include "px4_flight/flight_mode_manager.hpp"
#include "px4_flight/offboard_controller.hpp"
#include "px4_flight/command_mux.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // 创建各个模块
  auto px4_interface = std::make_shared<px4_flight::PX4Interface>(options);
  auto pose_bridge = std::make_shared<px4_flight::ExternalPoseBridge>(options);
  auto mode_manager = std::make_shared<px4_flight::FlightModeManager>(options);
  auto offboard_controller = std::make_shared<px4_flight::OffboardController>(options);
  auto command_mux = std::make_shared<px4_flight::CommandMux>(options);

  // 设置依赖关系
  pose_bridge->set_px4_interface(px4_interface);
  mode_manager->set_px4_interface(px4_interface);
  // 【移除】offboard_controller->set_px4_interface(px4_interface);
  // 【移除】offboard_controller->set_mode_manager(mode_manager);

  // 创建多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(px4_interface);
  executor.add_node(pose_bridge);
  executor.add_node(mode_manager);
  executor.add_node(offboard_controller);
  executor.add_node(command_mux);

  RCLCPP_INFO(px4_interface->get_logger(), "PX4 Flight System starting...");

  try {
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(px4_interface->get_logger(), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}

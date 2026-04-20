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

  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Creating nodes...");

    auto px4_interface = std::make_shared<px4_flight::PX4Interface>(options);
    RCLCPP_INFO(rclcpp::get_logger("main"), "PX4 Interface created");

    auto pose_bridge = std::make_shared<px4_flight::ExternalPoseBridge>(options);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Pose Bridge created");

    auto mode_manager = std::make_shared<px4_flight::FlightModeManager>(options);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Mode Manager created");

    auto offboard_controller = std::make_shared<px4_flight::OffboardController>(options);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Offboard Controller created");

    auto command_mux = std::make_shared<px4_flight::CommandMux>(options);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Command Mux created");

    // 设置依赖
    pose_bridge->set_px4_interface(px4_interface);
    mode_manager->set_px4_interface(px4_interface);
    offboard_controller->set_px4_interface(px4_interface);
    offboard_controller->set_mode_manager(mode_manager);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Dependencies set");

    // 使用单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(px4_interface);
    executor.add_node(pose_bridge);
    executor.add_node(mode_manager);
    executor.add_node(offboard_controller);
    executor.add_node(command_mux);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting executor...");
    executor.spin();

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown exception");
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

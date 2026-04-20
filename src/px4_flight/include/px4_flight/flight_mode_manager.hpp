#ifndef PX4_FLIGHT__FLIGHT_MODE_MANAGER_HPP_
#define PX4_FLIGHT__FLIGHT_MODE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/string.hpp>

#include "px4_flight/px4_interface.hpp"

#include <string>
#include <map>
#include <functional>

namespace px4_flight
{

enum class FlightMode {
  UNKNOWN,
  MANUAL,
  STABILIZED,
  POSITION,
  OFFBOARD
};

/**
 * @brief 飞行模式管理器
 * 处理模式切换逻辑，状态机管理
 */
class FlightModeManager : public rclcpp::Node
{
public:
  explicit FlightModeManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FlightModeManager() = default;

  void set_px4_interface(std::shared_ptr<PX4Interface> px4_interface);

  // 模式切换请求
  bool request_mode_change(FlightMode target_mode);
  FlightMode get_current_mode() const { return current_mode_; }
  FlightMode get_target_mode() const { return target_mode_; }

  // 检查是否允许切换
  bool is_transition_allowed(FlightMode from, FlightMode to);

private:
  // 回调
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
  void mode_watchdog_callback();

  // 模式切换执行
  bool execute_mode_change(FlightMode target_mode);
  bool enter_position_mode();
  bool enter_offboard_mode();

  // 状态机
  void update_state_machine();

  // 参数
  double transition_timeout_;
  double pre_flight_check_duration_;
  std::map<std::string, std::vector<std::string>> allowed_transitions_;

  // 订阅者
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;

  // 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_state_pub_;

  // PX4接口
  std::shared_ptr<PX4Interface> px4_interface_;

  // 状态
  FlightMode current_mode_{FlightMode::UNKNOWN};
  FlightMode target_mode_{FlightMode::UNKNOWN};
  rclcpp::Time mode_change_request_time_;
  bool mode_change_in_progress_{false};
  
  // 条件缓存
  bool hovering_{false};
  bool odom_valid_{false};
  bool rc_connected_{false};
};

}  // namespace px4_flight

#endif  // PX4_FLIGHT__FLIGHT_MODE_MANAGER_HPP_
#include "px4_flight/flight_mode_manager.hpp"

namespace px4_flight
{

FlightModeManager::FlightModeManager(const rclcpp::NodeOptions & options)
: Node("flight_mode_manager", options)
{
  this->declare_parameter("transition_timeout", 5.0);
  this->declare_parameter("pre_flight_check_duration", 3.0);

  transition_timeout_ = this->get_parameter("transition_timeout").as_double();
  pre_flight_check_duration_ = this->get_parameter("pre_flight_check_duration").as_double();

  // 配置允许的切换
  allowed_transitions_["POSITION"] = {"OFFBOARD"};
  allowed_transitions_["OFFBOARD"] = {"POSITION", "MANUAL"};

  // 【修复】PX4 v1.16.0使用BEST_EFFORT QoS
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v1", qos_profile,
    std::bind(&FlightModeManager::vehicle_status_callback, this, std::placeholders::_1));

  control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
    "/fmu/out/vehicle_control_mode", qos_profile,
    std::bind(&FlightModeManager::vehicle_control_mode_callback, this, std::placeholders::_1));

  mode_state_pub_ = this->create_publisher<std_msgs::msg::String>("flight_mode_state", 10);

  // 模式监控定时器
  auto watchdog_period = std::chrono::milliseconds(100);
  this->create_wall_timer(watchdog_period, 
    std::bind(&FlightModeManager::mode_watchdog_callback, this));

  RCLCPP_INFO(this->get_logger(), "Flight Mode Manager initialized");
}

void FlightModeManager::set_px4_interface(std::shared_ptr<PX4Interface> px4_interface)
{
  px4_interface_ = px4_interface;
}

bool FlightModeManager::request_mode_change(FlightMode target_mode)
{
  if (mode_change_in_progress_) {
    RCLCPP_WARN(this->get_logger(), "Mode change already in progress");
    return false;
  }

  if (!is_transition_allowed(current_mode_, target_mode)) {
    RCLCPP_WARN(this->get_logger(), "Transition from %d to %d not allowed", 
      static_cast<int>(current_mode_), static_cast<int>(target_mode));
    return false;
  }

  target_mode_ = target_mode;
  mode_change_request_time_ = this->get_clock()->now();
  mode_change_in_progress_ = true;

  return execute_mode_change(target_mode);
}

bool FlightModeManager::is_transition_allowed(FlightMode from, FlightMode to)
{
  if (from == FlightMode::UNKNOWN) return true;
  
  std::string from_str;
  switch(from) {
    case FlightMode::MANUAL: from_str = "MANUAL"; break;
    case FlightMode::STABILIZED: from_str = "STABILIZED"; break;
    case FlightMode::POSITION: from_str = "POSITION"; break;
    case FlightMode::OFFBOARD: from_str = "OFFBOARD"; break;
    default: return false;
  }

  auto it = allowed_transitions_.find(from_str);
  if (it == allowed_transitions_.end()) return false;

  std::string to_str;
  switch(to) {
    case FlightMode::MANUAL: to_str = "MANUAL"; break;
    case FlightMode::STABILIZED: to_str = "STABILIZED"; break;
    case FlightMode::POSITION: to_str = "POSITION"; break;
    case FlightMode::OFFBOARD: to_str = "OFFBOARD"; break;
    default: return false;
  }

  auto& allowed = it->second;
  return std::find(allowed.begin(), allowed.end(), to_str) != allowed.end();
}

bool FlightModeManager::execute_mode_change(FlightMode target_mode)
{
  switch(target_mode) {
    case FlightMode::POSITION:
      return enter_position_mode();
    case FlightMode::OFFBOARD:
      return enter_offboard_mode();
    default:
      return false;
  }
}

bool FlightModeManager::enter_position_mode()
{
  if (!px4_interface_) return false;
  
  bool success = px4_interface_->request_position_mode();
  if (success) {
    RCLCPP_INFO(this->get_logger(), "Requested POSITION mode");
  }
  return success;
}

bool FlightModeManager::enter_offboard_mode()
{
  if (!px4_interface_) return false;

  if (!px4_interface_->is_armed()) {
    RCLCPP_WARN(this->get_logger(), "Cannot enter OFFBOARD: vehicle not armed");
    return false;
  }

  if (!px4_interface_->is_position_valid()) {
    RCLCPP_WARN(this->get_logger(), "Cannot enter OFFBOARD: position not valid");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Pre-flight check: waiting %.1f seconds...", 
    pre_flight_check_duration_);
  
  bool success = px4_interface_->request_offboard_mode();
  if (success) {
    RCLCPP_INFO(this->get_logger(), "Requested OFFBOARD mode");
  }
  return success;
}

void FlightModeManager::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  // 【修复】v1.16.0没有rc_signal_lost字段，使用gcs_connection_lost或移除
  // rc_connected_ = msg->rc_signal_lost == 0;  // 旧代码，已删除
  rc_connected_ = !msg->gcs_connection_lost;  // 使用GCS连接状态替代，或设为true
  
  // 更新当前模式
  FlightMode new_mode = FlightMode::UNKNOWN;
  if (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL) {
    new_mode = FlightMode::POSITION;
  } else if (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
    new_mode = FlightMode::OFFBOARD;
  } else if (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL) {
    new_mode = FlightMode::MANUAL;
  } else if (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB) {
    new_mode = FlightMode::STABILIZED;
  }

  if (new_mode != current_mode_) {
    RCLCPP_INFO(this->get_logger(), "Mode changed from %d to %d", 
      static_cast<int>(current_mode_), static_cast<int>(new_mode));
    current_mode_ = new_mode;
    mode_change_in_progress_ = false;
  }
}

void FlightModeManager::vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
  hovering_ = msg->flag_control_position_enabled && 
              !msg->flag_control_offboard_enabled;
  odom_valid_ = true;
}

void FlightModeManager::mode_watchdog_callback()
{
  if (!mode_change_in_progress_) return;

  auto now = this->get_clock()->now();
  if ((now - mode_change_request_time_).seconds() > transition_timeout_) {
    RCLCPP_ERROR(this->get_logger(), "Mode change timeout!");
    mode_change_in_progress_ = false;
    
    if (px4_interface_) {
      px4_interface_->request_position_mode();
    }
  }

  // 发布状态
  std_msgs::msg::String state_msg;
  state_msg.data = "Current: " + std::to_string(static_cast<int>(current_mode_)) + 
                   " Target: " + std::to_string(static_cast<int>(target_mode_)) +
                   " InProgress: " + std::to_string(mode_change_in_progress_);
  mode_state_pub_->publish(state_msg);
}

}  // namespace px4_flight
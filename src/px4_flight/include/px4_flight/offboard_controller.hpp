#ifndef PX4_FLIGHT__OFFBOARD_CONTROLLER_HPP_
#define PX4_FLIGHT__OFFBOARD_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

#include "px4_flight/px4_interface.hpp"
#include "px4_flight/msg/external_command.hpp"
#include <deque>

namespace px4_flight
{

enum class ControlType {
  POSITION,
  VELOCITY,
  ACCELERATION
};

/**
 * @brief Offboard控制器
 * 接收外部控制指令，生成平滑轨迹，发送至PX4
 */
class OffboardController : public rclcpp::Node
{
public:
  explicit OffboardController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OffboardController() = default;

  void set_px4_interface(std::shared_ptr<PX4Interface> px4_interface);
  void set_mode_manager(std::shared_ptr<rclcpp::Node> mode_manager);

  // 激活/停用
  bool activate();
  void deactivate();
  bool is_active() const { return active_; }

private:
  // 指令回调
  void external_command_callback(const px4_flight::msg::ExternalCommand::SharedPtr msg);
  void pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void twist_command_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // 轨迹生成
  geometry_msgs::msg::PoseStamped generate_smooth_setpoint(
    const geometry_msgs::msg::PoseStamped::SharedPtr & target);
  
  void trajectory_timer_callback();

  // 边界检查
  bool check_geofence(const geometry_msgs::msg::PoseStamped::SharedPtr & pose);
  geometry_msgs::msg::PoseStamped apply_geofence(
    const geometry_msgs::msg::PoseStamped::SharedPtr & pose);

  // 参数
  double setpoint_rate_;
  double smoothing_duration_;
  double max_velocity_;
  double max_acceleration_;
  bool enable_geofence_;
  
  // 订阅者
  rclcpp::Subscription<px4_flight::msg::ExternalCommand>::SharedPtr external_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;

  // 发布者
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_setpoint_pub_;

  // PX4接口
  std::shared_ptr<PX4Interface> px4_interface_;

  // 状态
  std::atomic<bool> active_{false};
  geometry_msgs::msg::PoseStamped current_target_;
  geometry_msgs::msg::PoseStamped current_setpoint_;
  rclcpp::Time last_command_time_;
  double command_timeout_;

  // 轨迹生成
  rclcpp::TimerBase::SharedPtr trajectory_timer_;
  std::deque<geometry_msgs::msg::PoseStamped> trajectory_buffer_;
};

}  // namespace px4_flight

#endif  // PX4_FLIGHT__OFFBOARD_CONTROLLER_HPP_
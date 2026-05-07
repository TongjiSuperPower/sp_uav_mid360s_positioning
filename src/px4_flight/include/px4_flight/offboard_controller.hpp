#ifndef PX4_FLIGHT__OFFBOARD_CONTROLLER_HPP_
#define PX4_FLIGHT__OFFBOARD_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>        // 【修复1】VehicleStatus（非V1）
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <Eigen/Dense>
#include <deque>
#include <atomic>

namespace px4_flight
{

enum class LandingState {
  IDLE,
  POSITION_HOLD,
  OFFBOARD_ACTIVE,
  APPROACH_XY,
  HOVER_ABOVE,
  DESCEND,
  FINAL_LAND,
  LANDED
};

struct QRPose {
  double x;
  double y;
  double z;
  double yaw;
  bool valid;
  rclcpp::Time timestamp;
};

class OffboardController : public rclcpp::Node
{
public:
  explicit OffboardController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OffboardController() = default;

  bool activate();
  void deactivate();
  bool is_active() const { return active_; }

private:
  // 【修复2】使用VehicleStatus（非VehicleStatusV1）
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void qr_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void rc_command_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // 【修复3】添加声明
  void publish_offboard_heartbeat();

  void state_machine_update();
  void transition_to(LandingState new_state);

  geometry_msgs::msg::PoseStamped generate_approach_setpoint();
  geometry_msgs::msg::PoseStamped generate_hover_setpoint();
  geometry_msgs::msg::PoseStamped generate_descend_setpoint();
  geometry_msgs::msg::PoseStamped generate_land_setpoint();

  void publish_offboard_setpoint(const geometry_msgs::msg::PoseStamped& setpoint);
  void publish_offboard_setpoint(const Eigen::Vector3d& position, double yaw);
  void publish_offboard_velocity(const Eigen::Vector3d& velocity, double yaw_rate);

  bool check_timeout(const rclcpp::Time& last_time, double timeout_sec);
  bool is_landing_complete();

  // 参数
  double approach_xy_speed_;
  double descend_speed_;
  double final_land_speed_;
  double hover_height_;
  double target_height_;
  double xy_tolerance_;
  double yaw_tolerance_;
  double qr_timeout_;
  double offboard_timeout_;
  double max_horizontal_speed_;
  double max_vertical_speed_;

  // 订阅者
  // 【修复4】使用VehicleStatus
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qr_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rc_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;

  // 发布者
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_vis_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr offboard_heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  // 状态
  std::atomic<bool> active_{false};
  LandingState current_state_{LandingState::IDLE};
  LandingState previous_state_{LandingState::IDLE};

  bool armed_{false};
  bool offboard_mode_{false};
  bool position_valid_{false};
  double current_x_{0.0}, current_y_{0.0}, current_z_{0.0};
  double current_yaw_{0.0};

  QRPose qr_pose_;
  bool qr_detected_{false};
  rclcpp::Time last_qr_time_;

  Eigen::Vector3d target_position_ned_;
  double target_yaw_ned_{0.0};
  Eigen::Vector3d land_start_position_;
  double land_start_yaw_{0.0};

  rclcpp::Time state_entry_time_;
};

}  // namespace px4_flight

#endif  // PX4_FLIGHT__OFFBOARD_CONTROLLER_HPP_
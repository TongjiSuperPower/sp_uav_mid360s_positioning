#ifndef PX4_FLIGHT__PX4_INTERFACE_HPP_
#define PX4_FLIGHT__PX4_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <memory>
#include <atomic>
#include <mutex>

namespace px4_flight
{

class PX4Interface : public rclcpp::Node
{
public:
  explicit PX4Interface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PX4Interface() = default;

  bool is_connected() const { return connected_; }
  bool is_armed() const { return armed_; }
  bool is_offboard_mode() const { return offboard_mode_; }
  bool is_position_valid() const { return position_valid_; }
  
  px4_msgs::msg::VehicleStatus get_vehicle_status() const;
  // 移除：px4_msgs::msg::VehicleOdometry get_current_odometry() const;

  bool send_visual_odometry(const nav_msgs::msg::Odometry::SharedPtr & odom_msg);
  bool send_offboard_setpoint(const geometry_msgs::msg::PoseStamped::SharedPtr & pose_msg);
  bool send_offboard_setpoint(const geometry_msgs::msg::TwistStamped::SharedPtr & twist_msg);
  
  bool request_offboard_mode();
  bool request_position_mode();
  bool request_hold_mode();
  bool request_land();
  bool request_disarm();

private:
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
  // 移除：void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  void heartbeat_timer_callback();
  void offboard_heartbeat_callback();

  px4_msgs::msg::VehicleOdometry convert_enu_to_ned(
    const nav_msgs::msg::Odometry::SharedPtr & odom_msg);
  
  px4_msgs::msg::TrajectorySetpoint convert_setpoint_to_ned(
    const geometry_msgs::msg::PoseStamped::SharedPtr & pose_msg);

  std::string visual_odometry_topic_;
  std::string offboard_control_mode_topic_;
  std::string trajectory_setpoint_topic_;
  std::string vehicle_command_topic_;
  double heartbeat_rate_;
  double offboard_rate_;
  double timeout_threshold_;
  bool use_xrce_dds_;

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odometry_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;
  // 移除：rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr offboard_heartbeat_timer_;

  std::atomic<bool> connected_{false};
  std::atomic<bool> armed_{false};
  std::atomic<bool> offboard_mode_{false};
  std::atomic<bool> position_valid_{false};
  
  mutable std::mutex status_mutex_;
  px4_msgs::msg::VehicleStatus latest_status_;
  // 移除：px4_msgs::msg::VehicleOdometry latest_odometry_;
  px4_msgs::msg::VehicleLocalPosition latest_local_position_;
  rclcpp::Time last_heartbeat_time_;
};

}  // namespace px4_flight

#endif  // PX4_FLIGHT__PX4_INTERFACE_HPP_

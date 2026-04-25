#ifndef QR_LANDING__LANDING_CONTROLLER_HPP_
#define QR_LANDING__LANDING_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <algorithm>

namespace qr_landing
{

enum class OpMode {
  FOLLOW = 0,
  LAND
};

enum class FollowState {
  IDLE = 0,
  SEARCHING,
  TRACKING,
  LOST
};

enum class LandState {
  IDLE = 0,
  SEARCHING,
  APPROACHING,
  DESCENDING,
  FINAL_LANDING,
  LANDED,
  ABORTED
};

class LandingControllerNode : public rclcpp::Node
{
public:
  explicit LandingControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void qrRelativeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void activateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void modeCallback(const std_msgs::msg::String::SharedPtr msg);
  void controlLoop();
  
  // Follow mode
  void publishFollowIdle();
  void publishFollowSearch();
  void publishFollowTrack();
  
  // Land mode
  void publishLandIdle();
  void publishLandSearch();
  void publishLandApproach();
  void publishLandDescend();
  void publishLandFinal();
  void publishLandLanded();
  void publishLandAbort();
  
  void resetAllStates();
  std::string getFullStateString();
  
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qr_relative_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activate_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // Mode & State
  OpMode op_mode_ = OpMode::FOLLOW;
  FollowState follow_state_ = FollowState::IDLE;
  LandState land_state_ = LandState::IDLE;
  
  // QR data
  bool qr_detected_ = false;
  rclcpp::Time last_qr_time_;
  geometry_msgs::msg::PoseStamped qr_relative_;
  
  // Parameters
  double follow_altitude_;
  double follow_max_speed_;
  double approach_altitude_;
  double descend_start_altitude_;
  double final_altitude_;
  double landing_speed_;
  double horizontal_speed_;
  double max_horizontal_error_;
  double yaw_speed_;
  double qr_timeout_;
  double kp_xy_;
  double kp_yaw_;
  double kp_z_;
};

} // namespace qr_landing

#endif // QR_LANDING__LANDING_CONTROLLER_HPP_
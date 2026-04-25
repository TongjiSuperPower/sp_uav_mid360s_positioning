#include "qr_landing/landing_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace qr_landing
{

LandingControllerNode::LandingControllerNode(const rclcpp::NodeOptions & options)
: Node("landing_controller", options)
{
  this->declare_parameter("follow_altitude", 2.0);
  this->declare_parameter("follow_max_speed", 1.0);
  this->declare_parameter("approach_altitude", 2.0);
  this->declare_parameter("descend_start_altitude", 1.0);
  this->declare_parameter("final_altitude", 0.3);
  this->declare_parameter("landing_speed", 0.3);
  this->declare_parameter("horizontal_speed", 1.0);
  this->declare_parameter("max_horizontal_error", 0.08);
  this->declare_parameter("yaw_speed", 0.5);
  this->declare_parameter("qr_timeout", 2.0);
  this->declare_parameter("kp_xy", 0.6);
  this->declare_parameter("kp_yaw", 0.8);
  this->declare_parameter("kp_z", 0.5);
  this->declare_parameter("default_mode", "FOLLOW");
  
  follow_altitude_ = this->get_parameter("follow_altitude").as_double();
  follow_max_speed_ = this->get_parameter("follow_max_speed").as_double();
  approach_altitude_ = this->get_parameter("approach_altitude").as_double();
  descend_start_altitude_ = this->get_parameter("descend_start_altitude").as_double();
  final_altitude_ = this->get_parameter("final_altitude").as_double();
  landing_speed_ = this->get_parameter("landing_speed").as_double();
  horizontal_speed_ = this->get_parameter("horizontal_speed").as_double();
  max_horizontal_error_ = this->get_parameter("max_horizontal_error").as_double();
  yaw_speed_ = this->get_parameter("yaw_speed").as_double();
  qr_timeout_ = this->get_parameter("qr_timeout").as_double();
  kp_xy_ = this->get_parameter("kp_xy").as_double();
  kp_yaw_ = this->get_parameter("kp_yaw").as_double();
  kp_z_ = this->get_parameter("kp_z").as_double();
  
  std::string def_mode = this->get_parameter("default_mode").as_string();
  if (def_mode == "LAND") op_mode_ = OpMode::LAND;
  else op_mode_ = OpMode::FOLLOW;
  
  qr_relative_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/qr_landing/qr_relative", 10,
    std::bind(&LandingControllerNode::qrRelativeCallback, this, std::placeholders::_1));
    
  activate_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/qr_landing/activate", 10,
    std::bind(&LandingControllerNode::activateCallback, this, std::placeholders::_1));
    
  mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/qr_landing/mode", 10,
    std::bind(&LandingControllerNode::modeCallback, this, std::placeholders::_1));
  
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/qr_landing/cmd_velocity", 10);
  state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/qr_landing/state", 10);
  
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&LandingControllerNode::controlLoop, this));
  
  RCLCPP_INFO(this->get_logger(),
    "LandingController ready. Default mode: %s", def_mode.c_str());
}

void LandingControllerNode::qrRelativeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_qr_time_ = this->now();
  qr_detected_ = true;
  qr_relative_ = *msg;
}

void LandingControllerNode::activateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    if (op_mode_ == OpMode::FOLLOW && follow_state_ == FollowState::IDLE) {
      follow_state_ = FollowState::SEARCHING;
      RCLCPP_INFO(this->get_logger(), "ACTIVATED -> FOLLOW:SEARCHING");
    } else if (op_mode_ == OpMode::LAND && land_state_ == LandState::IDLE) {
      land_state_ = LandState::SEARCHING;
      RCLCPP_INFO(this->get_logger(), "ACTIVATED -> LAND:SEARCHING");
    }
  } else {
    resetAllStates();
    RCLCPP_INFO(this->get_logger(), "DEACTIVATED -> IDLE");
  }
}

void LandingControllerNode::modeCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "FOLLOW" && op_mode_ != OpMode::FOLLOW) {
    op_mode_ = OpMode::FOLLOW;
    land_state_ = LandState::IDLE;
    follow_state_ = FollowState::IDLE;
    RCLCPP_INFO(this->get_logger(), "Mode switched to FOLLOW");
  } else if (msg->data == "LAND" && op_mode_ != OpMode::LAND) {
    op_mode_ = OpMode::LAND;
    follow_state_ = FollowState::IDLE;
    land_state_ = LandState::IDLE;
    RCLCPP_INFO(this->get_logger(), "Mode switched to LAND");
  }
}

void LandingControllerNode::resetAllStates()
{
  follow_state_ = FollowState::IDLE;
  land_state_ = LandState::IDLE;
  qr_detected_ = false;
}

void LandingControllerNode::controlLoop()
{
  if (qr_detected_ && (this->now() - last_qr_time_).seconds() > qr_timeout_) {
    qr_detected_ = false;
    RCLCPP_WARN(this->get_logger(), "QR code lost (timeout)!");
    
    if (op_mode_ == OpMode::FOLLOW && follow_state_ == FollowState::TRACKING) {
      follow_state_ = FollowState::LOST;
    }
    if (op_mode_ == OpMode::LAND && 
        land_state_ != LandState::LANDED && 
        land_state_ != LandState::IDLE &&
        land_state_ != LandState::ABORTED) {
      land_state_ = LandState::ABORTED;
    }
  }
  
  if (op_mode_ == OpMode::FOLLOW) {
    switch (follow_state_) {
      case FollowState::IDLE:       publishFollowIdle(); break;
      case FollowState::SEARCHING:  publishFollowSearch(); break;
      case FollowState::TRACKING:   publishFollowTrack(); break;
      case FollowState::LOST:       publishFollowSearch(); break;
    }
  } else {
    switch (land_state_) {
      case LandState::IDLE:         publishLandIdle(); break;
      case LandState::SEARCHING:    publishLandSearch(); break;
      case LandState::APPROACHING:  publishLandApproach(); break;
      case LandState::DESCENDING:   publishLandDescend(); break;
      case LandState::FINAL_LANDING:publishLandFinal(); break;
      case LandState::LANDED:       publishLandLanded(); break;
      case LandState::ABORTED:      publishLandAbort(); break;
    }
  }
  
  auto state_msg = std::make_shared<std_msgs::msg::String>();
  state_msg->data = getFullStateString();
  state_pub_->publish(*state_msg);
}

void LandingControllerNode::publishFollowIdle()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;
  cmd.twist.linear.z = 0.0;
  cmd.twist.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

void LandingControllerNode::publishFollowSearch()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;
  cmd.twist.linear.z = 0.0;
  cmd.twist.angular.z = 0.3;
  cmd_vel_pub_->publish(cmd);
  
  if (qr_detected_) {
    follow_state_ = FollowState::TRACKING;
    RCLCPP_INFO(this->get_logger(), "FOLLOW: QR found -> TRACKING");
  }
}

void LandingControllerNode::publishFollowTrack()
{
  if (!qr_detected_) {
    follow_state_ = FollowState::LOST;
    return;
  }
  
  double dx = qr_relative_.pose.position.x;
  double dy = qr_relative_.pose.position.y;
  double height = -qr_relative_.pose.position.z;
  
  tf2::Quaternion q(
    qr_relative_.pose.orientation.x,
    qr_relative_.pose.orientation.y,
    qr_relative_.pose.orientation.z,
    qr_relative_.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  
  cmd.twist.linear.x = std::clamp(kp_xy_ * dx, -follow_max_speed_, follow_max_speed_);
  cmd.twist.linear.y = std::clamp(kp_xy_ * dy, -follow_max_speed_, follow_max_speed_);
  
  double alt_err = height - follow_altitude_;
  cmd.twist.linear.z = std::clamp(-kp_z_ * alt_err, -landing_speed_, landing_speed_);
  
  cmd.twist.angular.z = std::clamp(kp_yaw_ * yaw, -yaw_speed_, yaw_speed_);
  
  cmd_vel_pub_->publish(cmd);
}

void LandingControllerNode::publishLandIdle()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;
  cmd.twist.linear.z = 0.0;
  cmd.twist.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

void LandingControllerNode::publishLandSearch()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;
  cmd.twist.linear.z = 0.0;
  cmd.twist.angular.z = 0.3;
  cmd_vel_pub_->publish(cmd);
  
  if (qr_detected_) {
    land_state_ = LandState::APPROACHING;
    RCLCPP_INFO(this->get_logger(), "LAND: QR found -> APPROACHING");
  }
}

void LandingControllerNode::publishLandApproach()
{
  if (!qr_detected_) return;
  
  double dx = qr_relative_.pose.position.x;
  double dy = qr_relative_.pose.position.y;
  double height = -qr_relative_.pose.position.z;
  
  tf2::Quaternion q(
    qr_relative_.pose.orientation.x,
    qr_relative_.pose.orientation.y,
    qr_relative_.pose.orientation.z,
    qr_relative_.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  
  cmd.twist.linear.x = std::clamp(kp_xy_ * dx, -horizontal_speed_, horizontal_speed_);
  cmd.twist.linear.y = std::clamp(kp_xy_ * dy, -horizontal_speed_, horizontal_speed_);
  
  double alt_err = height - approach_altitude_;
  cmd.twist.linear.z = std::clamp(-kp_z_ * alt_err, -landing_speed_, landing_speed_);
  
  cmd.twist.angular.z = std::clamp(kp_yaw_ * yaw, -yaw_speed_, yaw_speed_);
  
  cmd_vel_pub_->publish(cmd);
  
  double horiz_err = std::sqrt(dx*dx + dy*dy);
  if (horiz_err < max_horizontal_error_ && std::abs(alt_err) < 0.2 && std::abs(yaw) < 0.1) {
    land_state_ = LandState::DESCENDING;
    RCLCPP_INFO(this->get_logger(), "LAND: Aligned at %.2fm -> DESCENDING", height);
  }
}

void LandingControllerNode::publishLandDescend()
{
  if (!qr_detected_) {
    land_state_ = LandState::ABORTED;
    return;
  }
  
  double dx = qr_relative_.pose.position.x;
  double dy = qr_relative_.pose.position.y;
  double height = -qr_relative_.pose.position.z;
  
  tf2::Quaternion q(
    qr_relative_.pose.orientation.x,
    qr_relative_.pose.orientation.y,
    qr_relative_.pose.orientation.z,
    qr_relative_.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  
  cmd.twist.linear.x = std::clamp(1.2 * kp_xy_ * dx, -horizontal_speed_, horizontal_speed_);
  cmd.twist.linear.y = std::clamp(1.2 * kp_xy_ * dy, -horizontal_speed_, horizontal_speed_);
  cmd.twist.linear.z = -landing_speed_;
  cmd.twist.angular.z = std::clamp(kp_yaw_ * yaw, -yaw_speed_, yaw_speed_);
  
  cmd_vel_pub_->publish(cmd);
  
  if (height < final_altitude_) {
    land_state_ = LandState::FINAL_LANDING;
    RCLCPP_INFO(this->get_logger(), "LAND: Height %.2fm < %.2fm -> FINAL_LANDING", height, final_altitude_);
  }
}

void LandingControllerNode::publishLandFinal()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  
  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;
  cmd.twist.linear.z = -0.1;
  cmd.twist.angular.z = 0.0;
  
  cmd_vel_pub_->publish(cmd);
  
  double height = qr_detected_ ? -qr_relative_.pose.position.z : 999.0;
  if (height < 0.05) {
    land_state_ = LandState::LANDED;
    RCLCPP_INFO(this->get_logger(), "LAND: LANDED!");
  }
}

void LandingControllerNode::publishLandLanded()
{
  publishLandIdle();
}

void LandingControllerNode::publishLandAbort()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;
  cmd.twist.linear.z = 0.0;
  cmd.twist.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

std::string LandingControllerNode::getFullStateString()
{
  if (op_mode_ == OpMode::FOLLOW) {
    switch (follow_state_) {
      case FollowState::IDLE: return "FOLLOW:IDLE";
      case FollowState::SEARCHING: return "FOLLOW:SEARCHING";
      case FollowState::TRACKING: return "FOLLOW:TRACKING";
      case FollowState::LOST: return "FOLLOW:LOST";
      default: return "FOLLOW:UNKNOWN";
    }
  } else {
    switch (land_state_) {
      case LandState::IDLE: return "LAND:IDLE";
      case LandState::SEARCHING: return "LAND:SEARCHING";
      case LandState::APPROACHING: return "LAND:APPROACHING";
      case LandState::DESCENDING: return "LAND:DESCENDING";
      case LandState::FINAL_LANDING: return "LAND:FINAL_LANDING";
      case LandState::LANDED: return "LAND:LANDED";
      case LandState::ABORTED: return "LAND:ABORTED";
      default: return "LAND:UNKNOWN";
    }
  }
}

} // namespace qr_landing

// --- 传统 main 函数入口 ---
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qr_landing::LandingControllerNode>());
  rclcpp::shutdown();
  return 0;
}
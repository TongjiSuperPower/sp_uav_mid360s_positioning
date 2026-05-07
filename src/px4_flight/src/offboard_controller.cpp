#include "px4_flight/offboard_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace px4_flight
{

OffboardController::OffboardController(const rclcpp::NodeOptions & options)
: Node("offboard_controller", options)
{
  this->declare_parameter("approach_xy_speed", 1.0);
  this->declare_parameter("descend_speed", 0.3);
  this->declare_parameter("final_land_speed", 0.15);
  this->declare_parameter("hover_height", 2.0);
  this->declare_parameter("target_height", 0.4);
  this->declare_parameter("xy_tolerance", 0.1);
  this->declare_parameter("yaw_tolerance", 0.1);
  this->declare_parameter("qr_timeout", 0.5);
  this->declare_parameter("offboard_timeout", 0.5);
  this->declare_parameter("max_horizontal_speed", 3.0);
  this->declare_parameter("max_vertical_speed", 1.0);
  this->declare_parameter("qr_topic", "/qr_landing/qr_pose_raw");
  this->declare_parameter("setpoint_rate", 50.0);

  approach_xy_speed_ = this->get_parameter("approach_xy_speed").as_double();
  descend_speed_ = this->get_parameter("descend_speed").as_double();
  final_land_speed_ = this->get_parameter("final_land_speed").as_double();
  hover_height_ = this->get_parameter("hover_height").as_double();
  target_height_ = this->get_parameter("target_height").as_double();
  xy_tolerance_ = this->get_parameter("xy_tolerance").as_double();
  yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
  qr_timeout_ = this->get_parameter("qr_timeout").as_double();
  offboard_timeout_ = this->get_parameter("offboard_timeout").as_double();
  max_horizontal_speed_ = this->get_parameter("max_horizontal_speed").as_double();
  max_vertical_speed_ = this->get_parameter("max_vertical_speed").as_double();
  std::string qr_topic = this->get_parameter("qr_topic").as_string();
  double setpoint_rate = this->get_parameter("setpoint_rate").as_double();

  // 【修复5】QoS配置，避免volatile关键字冲突
  rclcpp::QoS qos_sensor(rclcpp::KeepLast(10));
  qos_sensor.best_effort();
  qos_sensor.durability_volatile();  // 使用durability_volatile()替代volatile()

  // 【修复6】使用VehicleStatus
  vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v1", qos_sensor,
    std::bind(&OffboardController::vehicle_status_callback, this, std::placeholders::_1));

  control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
    "/fmu/out/vehicle_control_mode", qos_sensor,
    std::bind(&OffboardController::vehicle_control_mode_callback, this, std::placeholders::_1));

  local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", qos_sensor,
    std::bind(&OffboardController::local_position_callback, this, std::placeholders::_1));

  qr_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    qr_topic, 10,
    std::bind(&OffboardController::qr_pose_callback, this, std::placeholders::_1));

  rc_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/rc/offboard_request", 10,
    std::bind(&OffboardController::rc_command_callback, this, std::placeholders::_1));

  emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/emergency/stop", 10,
    std::bind(&OffboardController::emergency_stop_callback, this, std::placeholders::_1));

  setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/fmu/in/trajectory_setpoint_pose", 10);
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/fmu/in/trajectory_setpoint_velocity", 10);
  state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/offboard_controller/active", 10);
  target_vis_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/offboard_controller/target_vis", 10);

  auto heartbeat_period = std::chrono::milliseconds(static_cast<int>(1000.0 / setpoint_rate));
  offboard_heartbeat_timer_ = this->create_wall_timer(
    heartbeat_period, std::bind(&OffboardController::publish_offboard_heartbeat, this));
  offboard_heartbeat_timer_->cancel();

  auto state_period = std::chrono::milliseconds(100);
  state_machine_timer_ = this->create_wall_timer(
    state_period, std::bind(&OffboardController::state_machine_update, this));
  state_machine_timer_->cancel();

  RCLCPP_INFO(this->get_logger(), 
    "OffboardController initialized, QR topic: %s", qr_topic.c_str());
}

bool OffboardController::activate()
{
  if (active_) return true;
  active_ = true;
  offboard_heartbeat_timer_->reset();
  state_machine_timer_->reset();
  current_state_ = LandingState::IDLE;
  qr_detected_ = false;
  RCLCPP_INFO(this->get_logger(), "OffboardController activated");
  return true;
}

void OffboardController::deactivate()
{
  active_ = false;
  offboard_heartbeat_timer_->cancel();
  state_machine_timer_->cancel();
  transition_to(LandingState::IDLE);
  RCLCPP_INFO(this->get_logger(), "OffboardController deactivated");
}

// 【修复7】使用VehicleStatus
void OffboardController::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
}

void OffboardController::vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
  offboard_mode_ = msg->flag_control_offboard_enabled;
}

void OffboardController::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  position_valid_ = msg->xy_valid && msg->z_valid;
  current_x_ = msg->x;
  current_y_ = msg->y;
  current_z_ = msg->z;
  current_yaw_ = msg->heading;
}

void OffboardController::qr_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  qr_pose_.x = msg->pose.position.x;
  qr_pose_.y = msg->pose.position.y;
  qr_pose_.z = msg->pose.position.z;
  
  tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  qr_pose_.yaw = yaw;
  
  qr_pose_.valid = true;
  qr_pose_.timestamp = msg->header.stamp;
  last_qr_time_ = this->get_clock()->now();
  qr_detected_ = true;
}

void OffboardController::rc_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !active_) {
    activate();
    transition_to(LandingState::POSITION_HOLD);
    RCLCPP_INFO(this->get_logger(), "RC request: activate offboard landing");
  } else if (!msg->data && active_) {
    deactivate();
    RCLCPP_INFO(this->get_logger(), "RC request: deactivate offboard");
  }
}

void OffboardController::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    deactivate();
    geometry_msgs::msg::PoseStamped emergency_land;
    emergency_land.header.stamp = this->get_clock()->now();
    emergency_land.header.frame_id = "map";
    emergency_land.pose.position.x = current_x_;
    emergency_land.pose.position.y = current_y_;
    emergency_land.pose.position.z = 0.0;
    publish_offboard_setpoint(emergency_land);
    RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP!");
  }
}

// 【修复8】添加实现
void OffboardController::publish_offboard_heartbeat()
{
  // Offboard模式心跳保持
  // 实际发送由state_machine_update中的publish_offboard_setpoint完成
}

void OffboardController::state_machine_update()
{
  if (!active_) return;

  auto now = this->get_clock()->now();
  bool qr_timeout = check_timeout(last_qr_time_, qr_timeout_);
  if (qr_timeout && qr_detected_) {
    RCLCPP_WARN(this->get_logger(), "QR data timeout!");
    qr_detected_ = false;
  }

  switch (current_state_) {
    case LandingState::IDLE:
      break;

    case LandingState::POSITION_HOLD:
      if (offboard_mode_) {
        transition_to(LandingState::OFFBOARD_ACTIVE);
      }
      break;

    case LandingState::OFFBOARD_ACTIVE:
      if (qr_detected_) {
        transition_to(LandingState::APPROACH_XY);
      } else {
        publish_offboard_setpoint(
          Eigen::Vector3d(current_x_, current_y_, current_z_), current_yaw_);
      }
      break;

    case LandingState::APPROACH_XY:
      if (!qr_detected_) {
        publish_offboard_setpoint(
          Eigen::Vector3d(current_x_, current_y_, hover_height_), current_yaw_);
        break;
      }
      {
        auto setpoint = generate_approach_setpoint();
        publish_offboard_setpoint(setpoint);
        double dx = std::abs(target_position_ned_(0) - current_x_);
        double dy = std::abs(target_position_ned_(1) - current_y_);
        double dyaw = std::abs(target_yaw_ned_ - current_yaw_);
        while (dyaw > M_PI) dyaw -= 2 * M_PI;
        while (dyaw < -M_PI) dyaw += 2 * M_PI;
        if (dx < xy_tolerance_ && dy < xy_tolerance_ && std::abs(dyaw) < yaw_tolerance_) {
          transition_to(LandingState::HOVER_ABOVE);
        }
      }
      break;

    case LandingState::HOVER_ABOVE:
      {
        auto setpoint = generate_hover_setpoint();
        publish_offboard_setpoint(setpoint);
        double dt = (now - state_entry_time_).seconds();
        if (dt > 2.0) {
          transition_to(LandingState::DESCEND);
        }
      }
      break;

    case LandingState::DESCEND:
      if (!qr_detected_) {
        transition_to(LandingState::HOVER_ABOVE);
        break;
      }
      {
        auto setpoint = generate_descend_setpoint();
        publish_offboard_setpoint(setpoint);
        double height_above_target = current_z_ - target_position_ned_(2);
        if (height_above_target < target_height_ + 0.05) {
          transition_to(LandingState::FINAL_LAND);
        }
      }
      break;

    case LandingState::FINAL_LAND:
      {
        auto setpoint = generate_land_setpoint();
        publish_offboard_setpoint(setpoint);
        if (is_landing_complete()) {
          transition_to(LandingState::LANDED);
        }
      }
      break;

    case LandingState::LANDED:
      RCLCPP_INFO(this->get_logger(), "Landing complete!");
      deactivate();
      break;
  }

  std_msgs::msg::Bool state_msg;
  state_msg.data = (current_state_ != LandingState::IDLE);
  state_pub_->publish(state_msg);
}

void OffboardController::transition_to(LandingState new_state)
{
  if (current_state_ == new_state) return;
  previous_state_ = current_state_;
  current_state_ = new_state;
  state_entry_time_ = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), 
    "State: %d -> %d", static_cast<int>(previous_state_), static_cast<int>(new_state));
}

bool OffboardController::check_timeout(const rclcpp::Time& last_time, double timeout_sec)
{
  return (this->get_clock()->now() - last_time).seconds() > timeout_sec;
}

bool OffboardController::is_landing_complete()
{
  return current_z_ < 0.2 || std::abs(current_z_ - land_start_position_(2)) < 0.1;
}

geometry_msgs::msg::PoseStamped OffboardController::generate_approach_setpoint()
{
  double dx_body = qr_pose_.x;
  double dy_body = qr_pose_.y;
  // double dz_body = qr_pose_.z;  // 未使用，水平阶段

  double cos_yaw = std::cos(current_yaw_);
  double sin_yaw = std::sin(current_yaw_);

  double dx_ned = dx_body * cos_yaw - dy_body * sin_yaw;
  double dy_ned = dx_body * sin_yaw + dy_body * cos_yaw;

  target_position_ned_(0) = current_x_ - dx_ned;
  target_position_ned_(1) = current_y_ - dy_ned;
  target_position_ned_(2) = hover_height_;

  target_yaw_ned_ = current_yaw_ - qr_pose_.yaw;
  while (target_yaw_ned_ > M_PI) target_yaw_ned_ -= 2 * M_PI;
  while (target_yaw_ned_ < -M_PI) target_yaw_ned_ += 2 * M_PI;

  double kp = 0.5;
  double vx = kp * (target_position_ned_(0) - current_x_);
  double vy = kp * (target_position_ned_(1) - current_y_);
  // double vz = 0.0;  // 未使用，水平阶段

  double v_horiz = std::sqrt(vx*vx + vy*vy);
  if (v_horiz > approach_xy_speed_) {
    vx = vx / v_horiz * approach_xy_speed_;
    vy = vy / v_horiz * approach_xy_speed_;
  }

  double dt = 0.1;
  geometry_msgs::msg::PoseStamped setpoint;
  setpoint.header.stamp = this->get_clock()->now();
  setpoint.header.frame_id = "map";
  setpoint.pose.position.x = current_x_ + vx * dt;
  setpoint.pose.position.y = current_y_ + vy * dt;
  setpoint.pose.position.z = hover_height_;

  tf2::Quaternion q;
  q.setRPY(0, 0, target_yaw_ned_);
  setpoint.pose.orientation.x = q.x();
  setpoint.pose.orientation.y = q.y();
  setpoint.pose.orientation.z = q.z();
  setpoint.pose.orientation.w = q.w();

  return setpoint;
}

geometry_msgs::msg::PoseStamped OffboardController::generate_hover_setpoint()
{
  geometry_msgs::msg::PoseStamped setpoint;
  setpoint.header.stamp = this->get_clock()->now();
  setpoint.header.frame_id = "map";
  setpoint.pose.position.x = target_position_ned_(0);
  setpoint.pose.position.y = target_position_ned_(1);
  setpoint.pose.position.z = hover_height_;

  tf2::Quaternion q;
  q.setRPY(0, 0, target_yaw_ned_);
  setpoint.pose.orientation.x = q.x();
  setpoint.pose.orientation.y = q.y();
  setpoint.pose.orientation.z = q.z();
  setpoint.pose.orientation.w = q.w();

  return setpoint;
}

geometry_msgs::msg::PoseStamped OffboardController::generate_descend_setpoint()
{
  double vz = -descend_speed_;
  if (qr_detected_) {
    double height_error = qr_pose_.z - target_height_;
    vz = -descend_speed_ * (1.0 + 0.5 * height_error);
    if (vz < -max_vertical_speed_) vz = -max_vertical_speed_;
    if (vz > 0) vz = 0;
  }

  geometry_msgs::msg::PoseStamped setpoint;
  setpoint.header.stamp = this->get_clock()->now();
  setpoint.header.frame_id = "map";
  setpoint.pose.position.x = target_position_ned_(0);
  setpoint.pose.position.y = target_position_ned_(1);
  setpoint.pose.position.z = current_z_ + vz * 0.1;

  tf2::Quaternion q;
  q.setRPY(0, 0, target_yaw_ned_);
  setpoint.pose.orientation.x = q.x();
  setpoint.pose.orientation.y = q.y();
  setpoint.pose.orientation.z = q.z();
  setpoint.pose.orientation.w = q.w();

  return setpoint;
}

geometry_msgs::msg::PoseStamped OffboardController::generate_land_setpoint()
{
  static bool initialized = false;
  if (!initialized) {
    land_start_position_ = Eigen::Vector3d(current_x_, current_y_, current_z_);
    land_start_yaw_ = current_yaw_;
    initialized = true;
  }

  double vz = -final_land_speed_;

  geometry_msgs::msg::PoseStamped setpoint;
  setpoint.header.stamp = this->get_clock()->now();
  setpoint.header.frame_id = "map";
  setpoint.pose.position.x = land_start_position_(0);
  setpoint.pose.position.y = land_start_position_(1);
  setpoint.pose.position.z = current_z_ + vz * 0.1;

  tf2::Quaternion q;
  q.setRPY(0, 0, land_start_yaw_);
  setpoint.pose.orientation.x = q.x();
  setpoint.pose.orientation.y = q.y();
  setpoint.pose.orientation.z = q.z();
  setpoint.pose.orientation.w = q.w();

  return setpoint;
}

void OffboardController::publish_offboard_setpoint(const geometry_msgs::msg::PoseStamped& setpoint)
{
  setpoint_pub_->publish(setpoint);
}

void OffboardController::publish_offboard_setpoint(const Eigen::Vector3d& position, double yaw)
{
  geometry_msgs::msg::PoseStamped setpoint;
  setpoint.header.stamp = this->get_clock()->now();
  setpoint.header.frame_id = "map";
  setpoint.pose.position.x = position(0);
  setpoint.pose.position.y = position(1);
  setpoint.pose.position.z = position(2);

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  setpoint.pose.orientation.x = q.x();
  setpoint.pose.orientation.y = q.y();
  setpoint.pose.orientation.z = q.z();
  setpoint.pose.orientation.w = q.w();

  setpoint_pub_->publish(setpoint);
}

void OffboardController::publish_offboard_velocity(const Eigen::Vector3d& velocity, double yaw_rate)
{
  geometry_msgs::msg::TwistStamped vel_msg;
  vel_msg.header.stamp = this->get_clock()->now();
  vel_msg.header.frame_id = "map";
  vel_msg.twist.linear.x = velocity(0);
  vel_msg.twist.linear.y = velocity(1);
  vel_msg.twist.linear.z = velocity(2);
  vel_msg.twist.angular.z = yaw_rate;

  velocity_pub_->publish(vel_msg);
}

}  // namespace px4_flight
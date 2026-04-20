#include "px4_flight/px4_interface.hpp"

namespace px4_flight
{

PX4Interface::PX4Interface(const rclcpp::NodeOptions & options)
: Node("px4_interface", options)
{
  visual_odometry_topic_ = this->get_parameter_or("visual_odometry_topic", std::string("/fmu/in/vehicle_visual_odometry"));
  offboard_control_mode_topic_ = this->get_parameter_or("offboard_control_mode_topic", std::string("/fmu/in/offboard_control_mode"));
  trajectory_setpoint_topic_ = this->get_parameter_or("trajectory_setpoint_topic", std::string("/fmu/in/trajectory_setpoint"));
  vehicle_command_topic_ = this->get_parameter_or("vehicle_command_topic", std::string("/fmu/in/vehicle_command"));
  heartbeat_rate_ = this->get_parameter_or("heartbeat_rate", 1.0);
  offboard_rate_ = this->get_parameter_or("offboard_rate", 50.0);
  timeout_threshold_ = this->get_parameter_or("timeout_threshold", 3.0);
  use_xrce_dds_ = this->get_parameter_or("use_xrce_dds", true);

  // 发布者
  visual_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
    visual_odometry_topic_, 10);
  offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    offboard_control_mode_topic_, 10);
  trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    trajectory_setpoint_topic_, 10);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    vehicle_command_topic_, 10);

  // 【修复2】PX4 v1.16.0使用BEST_EFFORT QoS，需要显式设置
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v1", qos_profile,
    std::bind(&PX4Interface::vehicle_status_callback, this, std::placeholders::_1));
  
  vehicle_control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
    "/fmu/out/vehicle_control_mode", qos_profile,
    std::bind(&PX4Interface::vehicle_control_mode_callback, this, std::placeholders::_1));

  vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", qos_profile,
    std::bind(&PX4Interface::vehicle_local_position_callback, this, std::placeholders::_1));

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / heartbeat_rate_)),
    std::bind(&PX4Interface::heartbeat_timer_callback, this));

  offboard_heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / offboard_rate_)),
    std::bind(&PX4Interface::offboard_heartbeat_callback, this));
  offboard_heartbeat_timer_->cancel();

  last_heartbeat_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "PX4 Interface v1.16.0 initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribing to: /fmu/out/vehicle_status_v1");
}

px4_msgs::msg::VehicleStatus PX4Interface::get_vehicle_status() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return latest_status_;
}

bool PX4Interface::send_visual_odometry(const nav_msgs::msg::Odometry::SharedPtr & odom_msg)
{
  auto px4_odom = convert_enu_to_ned(odom_msg);
  visual_odometry_pub_->publish(px4_odom);
  return true;
}

bool PX4Interface::send_offboard_setpoint(const geometry_msgs::msg::PoseStamped::SharedPtr & pose_msg)
{
  if (!offboard_mode_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
      "Not in offboard mode, cannot send setpoint");
    return false;
  }
  
  auto setpoint = convert_setpoint_to_ned(pose_msg);
  trajectory_setpoint_pub_->publish(setpoint);
  return true;
}

bool PX4Interface::send_offboard_setpoint(const geometry_msgs::msg::TwistStamped::SharedPtr & twist_msg)
{
  if (!offboard_mode_) {
    return false;
  }

  px4_msgs::msg::TrajectorySetpoint setpoint;
  setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  
  setpoint.velocity[0] = twist_msg->twist.linear.y;
  setpoint.velocity[1] = twist_msg->twist.linear.x;
  setpoint.velocity[2] = -twist_msg->twist.linear.z;
  
  setpoint.yawspeed = -twist_msg->twist.angular.z;

  trajectory_setpoint_pub_->publish(setpoint);
  return true;
}

bool PX4Interface::request_offboard_mode()
{
  px4_msgs::msg::VehicleCommand cmd;
  cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  cmd.param1 = 1;
  cmd.param2 = 6;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 197;
  cmd.from_external = true;

  vehicle_command_pub_->publish(cmd);
  offboard_heartbeat_timer_->reset();
  
  RCLCPP_INFO(this->get_logger(), "Requested OFFBOARD mode");
  return true;
}

bool PX4Interface::request_position_mode()
{
  px4_msgs::msg::VehicleCommand cmd;
  cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  cmd.param1 = 1;
  cmd.param2 = 3;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 197;
  cmd.from_external = true;

  vehicle_command_pub_->publish(cmd);
  offboard_heartbeat_timer_->cancel();
  
  RCLCPP_INFO(this->get_logger(), "Requested POSITION mode");
  return true;
}

bool PX4Interface::request_hold_mode()
{
  px4_msgs::msg::VehicleCommand cmd;
  cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  cmd.param1 = 1;
  cmd.param2 = 4;
  cmd.param3 = 3;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 197;
  cmd.from_external = true;

  vehicle_command_pub_->publish(cmd);
  RCLCPP_INFO(this->get_logger(), "Requested HOLD mode");
  return true;
}

bool PX4Interface::request_land()
{
  px4_msgs::msg::VehicleCommand cmd;
  cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 197;
  cmd.from_external = true;

  vehicle_command_pub_->publish(cmd);
  return true;
}

bool PX4Interface::request_disarm()
{
  px4_msgs::msg::VehicleCommand cmd;
  cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  cmd.param1 = 0;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 197;
  cmd.from_external = true;

  vehicle_command_pub_->publish(cmd);
  return true;
}

void PX4Interface::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  latest_status_ = *msg;
  
  armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  
  auto now = this->get_clock()->now();
  if ((now - last_heartbeat_time_).seconds() < timeout_threshold_) {
    connected_ = true;
  }
}

void PX4Interface::vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
  offboard_mode_ = msg->flag_control_offboard_enabled;
}

void PX4Interface::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  latest_local_position_ = *msg;
  
  position_valid_ = msg->xy_valid && msg->z_valid;
}

void PX4Interface::heartbeat_timer_callback()
{
  last_heartbeat_time_ = this->get_clock()->now();
}

void PX4Interface::offboard_heartbeat_callback()
{
  px4_msgs::msg::OffboardControlMode offboard_msg;
  offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_msg.position = true;
  offboard_msg.velocity = true;
  offboard_msg.acceleration = false;
  offboard_msg.attitude = false;
  offboard_msg.body_rate = false;
  offboard_msg.thrust_and_torque = false;
  offboard_msg.direct_actuator = false;

  offboard_control_mode_pub_->publish(offboard_msg);
}

px4_msgs::msg::VehicleOdometry PX4Interface::convert_enu_to_ned(
  const nav_msgs::msg::Odometry::SharedPtr & odom_msg)
{
  px4_msgs::msg::VehicleOdometry px4_odom;
  px4_odom.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  px4_odom.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;

  px4_odom.position[0] = odom_msg->pose.pose.position.y;
  px4_odom.position[1] = odom_msg->pose.pose.position.x;
  px4_odom.position[2] = -odom_msg->pose.pose.position.z;

  tf2::Quaternion q_enu(
    odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,
    odom_msg->pose.pose.orientation.z,
    odom_msg->pose.pose.orientation.w);

  tf2::Quaternion q_ned_to_enu;
  q_ned_to_enu.setRPY(M_PI, 0, M_PI / 2);
  
  tf2::Quaternion q_ned = q_ned_to_enu * q_enu;
  q_ned.normalize();

  px4_odom.q[0] = q_ned.w();
  px4_odom.q[1] = q_ned.x();
  px4_odom.q[2] = q_ned.y();
  px4_odom.q[3] = q_ned.z();

  px4_odom.velocity[0] = odom_msg->twist.twist.linear.y;
  px4_odom.velocity[1] = odom_msg->twist.twist.linear.x;
  px4_odom.velocity[2] = -odom_msg->twist.twist.linear.z;

  px4_odom.angular_velocity[0] = odom_msg->twist.twist.angular.x;
  px4_odom.angular_velocity[1] = odom_msg->twist.twist.angular.y;
  px4_odom.angular_velocity[2] = -odom_msg->twist.twist.angular.z;

  px4_odom.position_variance[0] = odom_msg->pose.covariance[0];
  px4_odom.position_variance[1] = odom_msg->pose.covariance[7];
  px4_odom.position_variance[2] = odom_msg->pose.covariance[14];
  
  px4_odom.orientation_variance[0] = odom_msg->pose.covariance[21];
  px4_odom.orientation_variance[1] = odom_msg->pose.covariance[28];
  px4_odom.orientation_variance[2] = odom_msg->pose.covariance[35];

  px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;

  px4_odom.quality = 1;

  return px4_odom;
}

px4_msgs::msg::TrajectorySetpoint PX4Interface::convert_setpoint_to_ned(
  const geometry_msgs::msg::PoseStamped::SharedPtr & pose_msg)
{
  px4_msgs::msg::TrajectorySetpoint setpoint;
  setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  setpoint.position[0] = pose_msg->pose.position.y;
  setpoint.position[1] = pose_msg->pose.position.x;
  setpoint.position[2] = -pose_msg->pose.position.z;

  setpoint.velocity[0] = 0;
  setpoint.velocity[1] = 0;
  setpoint.velocity[2] = 0;

  setpoint.acceleration[0] = 0;
  setpoint.acceleration[1] = 0;
  setpoint.acceleration[2] = 0;

  tf2::Quaternion q(
    pose_msg->pose.orientation.x,
    pose_msg->pose.orientation.y,
    pose_msg->pose.orientation.z,
    pose_msg->pose.orientation.w);
  
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  setpoint.yaw = -yaw;
  setpoint.yawspeed = 0;

  return setpoint;
}

}  // namespace px4_flight
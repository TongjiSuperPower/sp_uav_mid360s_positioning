#include "px4_flight/px4_interface.hpp"

namespace px4_flight
{

PX4Interface::PX4Interface(const rclcpp::NodeOptions & options)
: Node("px4_interface", options)
{
  // 声明参数
  this->declare_parameter("visual_odometry_topic", "/fmu/in/vehicle_visual_odometry");
  this->declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
  this->declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
  this->declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command");
  this->declare_parameter("heartbeat_rate", 1.0);
  this->declare_parameter("offboard_rate", 50.0);
  this->declare_parameter("timeout_threshold", 3.0);
  this->declare_parameter("use_xrce_dds", true);

  // 获取参数
  visual_odometry_topic_ = this->get_parameter("visual_odometry_topic").as_string();
  offboard_control_mode_topic_ = this->get_parameter("offboard_control_mode_topic").as_string();
  trajectory_setpoint_topic_ = this->get_parameter("trajectory_setpoint_topic").as_string();
  vehicle_command_topic_ = this->get_parameter("vehicle_command_topic").as_string();
  heartbeat_rate_ = this->get_parameter("heartbeat_rate").as_double();
  offboard_rate_ = this->get_parameter("offboard_rate").as_double();
  timeout_threshold_ = this->get_parameter("timeout_threshold").as_double();
  use_xrce_dds_ = this->get_parameter("use_xrce_dds").as_bool();

  // 发布者
  visual_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
    visual_odometry_topic_, 10);
  offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    offboard_control_mode_topic_, 10);
  trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    trajectory_setpoint_topic_, 10);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    vehicle_command_topic_, 10);

  // 【修复1】PX4 v1.16.0使用BEST_EFFORT QoS
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // 【修复2】订阅话题添加_v1后缀（v1.16.0）
  vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v1", qos_profile,
    std::bind(&PX4Interface::vehicle_status_callback, this, std::placeholders::_1));
  
  vehicle_control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
    "/fmu/out/vehicle_control_mode", qos_profile,
    std::bind(&PX4Interface::vehicle_control_mode_callback, this, std::placeholders::_1));

  vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", qos_profile,
    std::bind(&PX4Interface::vehicle_local_position_callback, this, std::placeholders::_1));

  // 定时器
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / heartbeat_rate_)),
    std::bind(&PX4Interface::heartbeat_timer_callback, this));

  offboard_heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / offboard_rate_)),
    std::bind(&PX4Interface::offboard_heartbeat_callback, this));
  offboard_heartbeat_timer_->cancel();

  // 【修复3】初始化时间戳
  last_heartbeat_time_ = this->get_clock()->now();
  last_yaw_valid_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "PX4 Interface v1.16.0 initialized");
  RCLCPP_INFO(this->get_logger(), "Visual odometry topic: %s", visual_odometry_topic_.c_str());
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

// 【修复4】核心：正确的ENU→NED四元数转换+航向稳定性
px4_msgs::msg::VehicleOdometry PX4Interface::convert_enu_to_ned(
  const nav_msgs::msg::Odometry::SharedPtr & odom_msg)
{
  px4_msgs::msg::VehicleOdometry px4_odom;
  
  // 使用ROS时间戳（统一时间源）
  auto now = this->get_clock()->now();
  px4_odom.timestamp = now.nanoseconds() / 1000;
  px4_odom.timestamp_sample = now.nanoseconds() / 1000;

  // ========== 位置转换 雷达机体坐标系 → NED ==========
  // Fast-LIO2输出坐标系: X=前, Y=左, Z=上
  // NED坐标系: X=北, Y=东, Z=下
  
  // 修正映射:
  // 雷达X(前) → NED_X(北): 直接映射
  // 雷达Y(左) → NED_Y(东): 左是东的负方向，取反
  // 雷达Z(上) → NED_Z(下): 上变下，取反
  
  px4_odom.position[0] = odom_msg->pose.pose.position.x;     // 前→北
  px4_odom.position[1] = odom_msg->pose.pose.position.y;    // 左→东 (左=-东)
  px4_odom.position[2] = -odom_msg->pose.pose.position.z;   // 上→下

  // ========== 姿态转换 雷达机体坐标系 → NED ==========
  // Fast-LIO2四元数基于机体坐标系
  // 需要转换到NED坐标系
  
  tf2::Quaternion q_body(
    odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,
    odom_msg->pose.pose.orientation.z,
    odom_msg->pose.pose.orientation.w);

  // 提取机体欧拉角
  double body_roll, body_pitch, body_yaw;
  tf2::Matrix3x3(q_body).getRPY(body_roll, body_pitch, body_yaw);
  
  // 机体→NED欧拉角转换
  // 机体前=0° → NED北=0°
  // 机体左倾=正roll → NED东倾=负roll（取反）
  // 机体抬头=正pitch → NED抬头=负pitch（NED下为正，取反）
  // 机体左偏航=正yaw → NED左偏航=负yaw（NED顺时针为正，取反）
  
  double ned_roll = -body_roll;      // 横滚取反
  double ned_pitch = -body_pitch;    // 俯仰取反
  double ned_yaw = -body_yaw;        // 航向取反
  
  // 归一化到[-π, π]
  while (ned_yaw > M_PI) ned_yaw -= 2 * M_PI;
  while (ned_yaw < -M_PI) ned_yaw += 2 * M_PI;

  // 航向稳定性检查（防止跳变导致EKF2拒绝）
  static double last_ned_yaw = 0.0;
  static bool yaw_initialized = false;
  static int yaw_reject_count = 0;
  const int YAW_REJECT_THRESHOLD = 5;
  
  if (!yaw_initialized) {
    last_ned_yaw = ned_yaw;
    yaw_initialized = true;
    RCLCPP_INFO(this->get_logger(), "Yaw initialized: %.2f°", ned_yaw * 180 / M_PI);
  }
  
  // 计算航向差（考虑圆周期）
  double yaw_diff = std::abs(ned_yaw - last_ned_yaw);
  if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;
  
  // 检测跳变（>30°认为异常）
  if (yaw_diff > M_PI / 6) {  // 30°
    yaw_reject_count++;
    RCLCPP_WARN(this->get_logger(), 
      "Yaw jump detected: %.1f° -> %.1f° (diff=%.1f°), reject_count=%d/%d",
      last_ned_yaw * 180 / M_PI, ned_yaw * 180 / M_PI, 
      yaw_diff * 180 / M_PI, yaw_reject_count, YAW_REJECT_THRESHOLD);
    
    if (yaw_reject_count < YAW_REJECT_THRESHOLD) {
      ned_yaw = last_ned_yaw + (ned_yaw - last_ned_yaw) * 0.1;  // 10%平滑
    } else {
      ned_yaw = last_ned_yaw;  // 强制保持
    }
  } else {
    yaw_reject_count = 0;
    last_ned_yaw = ned_yaw;
  }

  // 重建NED四元数
  tf2::Quaternion q_ned;
  q_ned.setRPY(ned_roll, ned_pitch, ned_yaw);
  q_ned.normalize();

  // PX4 VehicleOdometry: q = [w, x, y, z]
  px4_odom.q[0] = q_ned.w();
  px4_odom.q[1] = q_ned.x();
  px4_odom.q[2] = q_ned.y();
  px4_odom.q[3] = q_ned.z();

  // ========== 速度转换 雷达机体坐标系 → NED ==========
  // 与位置相同映射
  px4_odom.velocity[0] = odom_msg->twist.twist.linear.x;      // 前→北
  px4_odom.velocity[1] = odom_msg->twist.twist.linear.y;     // 左→东
  px4_odom.velocity[2] = -odom_msg->twist.twist.linear.z;     // 上→下

  // ========== 角速度转换 ==========
  // 角速度同样取反（因为坐标系手性变化）
  px4_odom.angular_velocity[0] = -odom_msg->twist.twist.angular.x;  // roll取反
  px4_odom.angular_velocity[1] = -odom_msg->twist.twist.angular.y;  // pitch取反
  px4_odom.angular_velocity[2] = -odom_msg->twist.twist.angular.z;   // yaw取反

  // ========== 协方差 ==========
  // 注意：协方差矩阵也需要相应旋转
  // 简化处理：直接使用对角线元素
  px4_odom.position_variance[0] = odom_msg->pose.covariance[0];   // x方差
  px4_odom.position_variance[1] = odom_msg->pose.covariance[7];   // y方差
  px4_odom.position_variance[2] = odom_msg->pose.covariance[14];  // z方差
  
  px4_odom.orientation_variance[0] = odom_msg->pose.covariance[21];  // roll方差
  px4_odom.orientation_variance[1] = odom_msg->pose.covariance[28];  // pitch方差
  px4_odom.orientation_variance[2] = odom_msg->pose.covariance[35];  // yaw方差

  // ========== 参考系设置（v1.16.0）==========
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
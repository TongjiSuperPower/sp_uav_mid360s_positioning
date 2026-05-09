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
  // ========== 【新增】发布频率限制（10Hz）==========
  static rclcpp::Time last_pub_time;
  auto now = this->get_clock()->now();
  
  if (last_pub_time.nanoseconds() > 0) {
    double dt = (now - last_pub_time).seconds();
    if (dt < 0.1) {
      return true;  // 跳过，避免EKF2过载
    }
  }
  last_pub_time = now;
  
  // ========== 原有转换代码 ==========
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
// 【借鉴MAVROS2】正确的ENU→NED四元数转换

px4_msgs::msg::VehicleOdometry PX4Interface::convert_enu_to_ned(
  const nav_msgs::msg::Odometry::SharedPtr & odom_msg)
{
  px4_msgs::msg::VehicleOdometry px4_odom;
  
  // ========== 时间戳（统一使用ROS时间，微秒）==========
  auto now = this->get_clock()->now();
  
  uint64_t now_us = now.nanoseconds() / 1000;
  px4_odom.timestamp = now_us;
  
  rclcpp::Time msg_time(odom_msg->header.stamp);
  uint64_t sample_us = msg_time.nanoseconds() / 1000;
  
  if (sample_us > now_us) {
    sample_us = now_us;
  }
  
  px4_odom.timestamp_sample = sample_us;

  // ========== 位置转换 ENU→NED ==========
  // ENU: X=东, Y=北, Z=天
  // NED: X=北, Y=东, Z=地
  px4_odom.position[0] = odom_msg->pose.pose.position.y;   // NED_X = ENU_Y (北) ✓
  px4_odom.position[1] = odom_msg->pose.pose.position.x;   // NED_Y = ENU_X (东) ✓
  px4_odom.position[2] = -odom_msg->pose.pose.position.z;  // NED_Z = -ENU_Z (地) ✓

  // ========== 【关键修正】姿态转换 ENU→NED ==========
  
  tf2::Quaternion q_enu(
    odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,
    odom_msg->pose.pose.orientation.z,
    odom_msg->pose.pose.orientation.w);

  // 检查有效性
  double norm = std::sqrt(q_enu.x()*q_enu.x() + q_enu.y()*q_enu.y() + 
                          q_enu.z()*q_enu.z() + q_enu.w()*q_enu.w());
  if (std::abs(norm - 1.0) > 0.01 || std::isnan(norm)) {
    q_enu = tf2::Quaternion(0, 0, 0, 1);
  } else {
    q_enu.normalize();
  }

  // ENU到NED欧拉角
  double roll_enu, pitch_enu, yaw_enu;
  tf2::Matrix3x3(q_enu).getRPY(roll_enu, pitch_enu, yaw_enu);
  
  // 【关键】航向映射：ENU逆时针正 → NED顺时针正，需取反
  double roll_ned = roll_enu;       // 滚转同向 ✓
  double pitch_ned = pitch_enu;     // 俯仰同向 ✓
  double yaw_ned = -yaw_enu;        // 航向反向（手性修正）✓

  // ========== 【关键】航向低通滤波（抗晃动）==========
  
  static double filtered_yaw = 0.0;
  static bool yaw_init = false;
  
  if (!yaw_init) {
    filtered_yaw = yaw_ned;
    yaw_init = true;
    RCLCPP_INFO(this->get_logger(), 
      "坐标对齐: ENU→NED, init yaw=%.2f°", yaw_ned * 180.0 / M_PI);
  } else {
    double diff = yaw_ned - filtered_yaw;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    
    // 自适应滤波：静止时快速跟随，运动时强滤波
    static double last_pos_x = 0.0, last_pos_y = 0.0;
    double pos_change = std::sqrt(
      std::pow(px4_odom.position[0] - last_pos_x, 2) +
      std::pow(px4_odom.position[1] - last_pos_y, 2));
    last_pos_x = px4_odom.position[0];
    last_pos_y = px4_odom.position[1];
    
    double alpha = (pos_change > 0.01) ? 0.05 : 0.3;
    filtered_yaw += alpha * diff;
    
    while (filtered_yaw > M_PI) filtered_yaw -= 2.0 * M_PI;
    while (filtered_yaw < -M_PI) filtered_yaw += 2.0 * M_PI;
  }

  // 重建四元数
  tf2::Quaternion q_ned;
  q_ned.setRPY(roll_ned, pitch_ned, filtered_yaw);
  q_ned.normalize();

  px4_odom.q[0] = q_ned.w();
  px4_odom.q[1] = q_ned.x();
  px4_odom.q[2] = q_ned.y();
  px4_odom.q[3] = q_ned.z();

  // ========== 【方案2新增】位置跳变检测与保护 ==========
  
  static double last_valid_x = 0.0, last_valid_y = 0.0, last_valid_z = 0.0;
  static bool pos_init = false;
  static rclcpp::Time last_pos_time;
  
  double curr_x = px4_odom.position[0];
  double curr_y = px4_odom.position[1];
  double curr_z = px4_odom.position[2];
  
  if (!pos_init) {
    // 首次初始化
    last_valid_x = curr_x;
    last_valid_y = curr_y;
    last_valid_z = curr_z;
    last_pos_time = now;
    pos_init = true;
    RCLCPP_INFO(this->get_logger(), 
      "位置初始化: %.3f, %.3f, %.3f", curr_x, curr_y, curr_z);
  } else {
    double dt = (now - last_pos_time).seconds();
    last_pos_time = now;
    
    // 计算帧间位移
    double dx = curr_x - last_valid_x;
    double dy = curr_y - last_valid_y;
    double dz = curr_z - last_valid_z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // 计算速度
    double velocity = dist / std::max(dt, 0.001);
    
    // 【关键】最大可信速度（根据实际调整，室内2m/s，室外5m/s）
    double max_velocity = 3.0;  // m/s
    
    if (velocity > max_velocity) {
      // 跳变 detected，使用预测位置而非Fast-LIO2输出
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "位置跳变: %.2fm/%.3fs=%.1fm/s > %.1fm/s, 使用预测",
        dist, dt, velocity, max_velocity);
      
      // 使用上一帧有效位置（保持静止预测）
      px4_odom.position[0] = last_valid_x;
      px4_odom.position[1] = last_valid_y;
      px4_odom.position[2] = last_valid_z;
      
      // 标记质量差，EKF2会降低权重但不会完全拒绝
      px4_odom.quality = 0;
      
      // 协方差增大，表示不确定性
      px4_odom.position_variance[0] = 1.0;
      px4_odom.position_variance[1] = 1.0;
      px4_odom.position_variance[2] = 1.0;
    } else {
      // 正常，更新上一帧有效位置
      last_valid_x = curr_x;
      last_valid_y = curr_y;
      last_valid_z = curr_z;
      px4_odom.quality = 1;
      
      // 正常协方差
      px4_odom.position_variance[0] = 0.01;
      px4_odom.position_variance[1] = 0.01;
      px4_odom.position_variance[2] = 0.04;
    }
  }

  // ========== 速度转换 ENU→NED ==========
  // 【注意】odom_msg->twist.twist.linear是世界坐标系下的速度
  px4_odom.velocity[0] = odom_msg->twist.twist.linear.y;   // NED_vx = ENU_vy (北向速度) ✓
  px4_odom.velocity[1] = odom_msg->twist.twist.linear.x;   // NED_vy = ENU_vx (东向速度) ✓
  px4_odom.velocity[2] = -odom_msg->twist.twist.linear.z;  // NED_vz = -ENU_vz (地向速度) ✓

  // ========== 角速度转换 ==========
  // 机体坐标系下的角速度
  px4_odom.angular_velocity[0] = odom_msg->twist.twist.angular.x;   // Roll变化率 ✓
  px4_odom.angular_velocity[1] = odom_msg->twist.twist.angular.y;   // Pitch变化率 ✓
  px4_odom.angular_velocity[2] = -odom_msg->twist.twist.angular.z;  // Yaw变化率（取反）✓

  // ========== 姿态协方差 ==========
  px4_odom.orientation_variance[0] = 0.01;   // roll
  px4_odom.orientation_variance[1] = 0.01;   // pitch
  px4_odom.orientation_variance[2] = 0.01;   // yaw

  // ========== 参考帧 ==========
  px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  
  // 【修正】速度是世界坐标系，应设为NED
  px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

  // 调试日志（每100帧）
  static int cnt = 0;
  if (++cnt % 100 == 0) {
    RCLCPP_INFO(this->get_logger(), 
      "对齐: raw_yaw=%.1f°, filt_yaw=%.1f°, pos=%.2f,%.2f,%.2f, qual=%d",
      yaw_enu * 180.0 / M_PI, filtered_yaw * 180.0 / M_PI,
      px4_odom.position[0], px4_odom.position[1], px4_odom.position[2],
      px4_odom.quality);
  }

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
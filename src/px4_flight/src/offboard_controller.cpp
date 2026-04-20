#include "px4_flight/offboard_controller.hpp"

namespace px4_flight
{

OffboardController::OffboardController(const rclcpp::NodeOptions & options)
: Node("offboard_controller", options)
{
  this->declare_parameter("setpoint_rate", 50.0);
  this->declare_parameter("smoothing_duration", 1.0);
  this->declare_parameter("max_velocity", 5.0);
  this->declare_parameter("max_acceleration", 2.0);
  this->declare_parameter("enable_geofence", true);
  this->declare_parameter("command_timeout", 0.5);

  setpoint_rate_ = this->get_parameter("setpoint_rate").as_double();
  smoothing_duration_ = this->get_parameter("smoothing_duration").as_double();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
  max_acceleration_ = this->get_parameter("max_acceleration").as_double();
  enable_geofence_ = this->get_parameter("enable_geofence").as_bool();
  command_timeout_ = this->get_parameter("command_timeout").as_double();

  // 订阅者
  external_cmd_sub_ = this->create_subscription<px4_flight::msg::ExternalCommand>(
    "/external/cmd", 10,
    std::bind(&OffboardController::external_command_callback, this, std::placeholders::_1));

  pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/external/pose_cmd", 10,
    std::bind(&OffboardController::pose_command_callback, this, std::placeholders::_1));

  twist_cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/external/twist_cmd", 10,
    std::bind(&OffboardController::twist_command_callback, this, std::placeholders::_1));

  emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/emergency/stop", 10,
    std::bind(&OffboardController::emergency_stop_callback, this, std::placeholders::_1));

  // 发布者
  trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
  current_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_setpoint", 10);

  // 轨迹定时器
  auto trajectory_period = std::chrono::milliseconds(static_cast<int>(1000.0 / setpoint_rate_));
  trajectory_timer_ = this->create_wall_timer(
    trajectory_period, std::bind(&OffboardController::trajectory_timer_callback, this));
  trajectory_timer_->cancel();  // 初始不启动

  RCLCPP_INFO(this->get_logger(), "Offboard Controller initialized");
}

void OffboardController::set_px4_interface(std::shared_ptr<PX4Interface> px4_interface)
{
  px4_interface_ = px4_interface;
}

void OffboardController::set_mode_manager(std::shared_ptr<rclcpp::Node> mode_manager)
{
  (void)mode_manager;
}

bool OffboardController::activate()
{
  if (active_) return true;
  
  active_ = true;
  trajectory_timer_->reset();
  RCLCPP_INFO(this->get_logger(), "Offboard Controller activated");
  return true;
}

void OffboardController::deactivate()
{
  active_ = false;
  trajectory_timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "Offboard Controller deactivated");
}

void OffboardController::external_command_callback(const px4_flight::msg::ExternalCommand::SharedPtr msg)
{
  if (!active_) return;

  last_command_time_ = this->get_clock()->now();

  geometry_msgs::msg::PoseStamped target;
  target.header = msg->header;
  target.pose = msg->pose;

  if (enable_geofence_ && !check_geofence(std::make_shared<geometry_msgs::msg::PoseStamped>(target))) {
    target = apply_geofence(std::make_shared<geometry_msgs::msg::PoseStamped>(target));
  }

  current_target_ = target;
}

void OffboardController::pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!active_) return;

  px4_flight::msg::ExternalCommand cmd;
  cmd.header = msg->header;
  cmd.pose = msg->pose;
  cmd.control_type = px4_flight::msg::ExternalCommand::POSITION;
  
  external_command_callback(std::make_shared<px4_flight::msg::ExternalCommand>(cmd));
}

void OffboardController::twist_command_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!active_) return;

  // 速度控制模式：需要积分得到位置设定点，或直接使用速度设定点
  // 这里简化为发布速度设定点
  if (px4_interface_) {
    px4_interface_->send_offboard_setpoint(msg);
  }
}

void OffboardController::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    deactivate();
    if (px4_interface_) {
      px4_interface_->request_position_mode();
    }
    RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
  }
}

void OffboardController::trajectory_timer_callback()
{
  if (!active_ || !px4_interface_) return;

  // 检查指令超时
  auto now = this->get_clock()->now();
  if ((now - last_command_time_).seconds() > command_timeout_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
      "Command timeout, holding position");
    // 保持当前设定点
    return;
  }

  // 生成平滑轨迹
  auto setpoint = generate_smooth_setpoint(
    std::make_shared<geometry_msgs::msg::PoseStamped>(current_target_));

  // 发送至PX4
  auto setpoint_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>(setpoint);
  px4_interface_->send_offboard_setpoint(setpoint_ptr);

  // 发布调试信息
  current_setpoint_pub_->publish(setpoint);
}

geometry_msgs::msg::PoseStamped OffboardController::generate_smooth_setpoint(
  const geometry_msgs::msg::PoseStamped::SharedPtr & target)
{
  // 简单的线性插值平滑
  geometry_msgs::msg::PoseStamped setpoint;
  setpoint.header.stamp = this->get_clock()->now();
  setpoint.header.frame_id = target->header.frame_id;

  // 当前位置（应从PX4反馈获取，简化使用内部状态）
  static geometry_msgs::msg::Pose current_pose;
  
  // 插值系数（根据平滑时间和控制频率计算）
  double alpha = 1.0 / (smoothing_duration_ * setpoint_rate_);
  if (alpha > 1.0) alpha = 1.0;

  setpoint.pose.position.x = current_pose.position.x + 
    alpha * (target->pose.position.x - current_pose.position.x);
  setpoint.pose.position.y = current_pose.position.y + 
    alpha * (target->pose.position.y - current_pose.position.y);
  setpoint.pose.position.z = current_pose.position.z + 
    alpha * (target->pose.position.z - current_pose.position.z);

  // 姿态使用目标值（或SLERP插值）
  setpoint.pose.orientation = target->pose.orientation;

  current_pose = setpoint.pose;
  return setpoint;
}

bool OffboardController::check_geofence(const geometry_msgs::msg::PoseStamped::SharedPtr & pose)
{
  // 简化的圆柱体围栏检查
  double distance = std::sqrt(
    pose->pose.position.x * pose->pose.position.x + 
    pose->pose.position.y * pose->pose.position.y);
  
  if (distance > 50.0) return false;  // 水平限制50m
  if (pose->pose.position.z > 30.0 || pose->pose.position.z < 0.5) return false;  // 高度限制

  return true;
}

geometry_msgs::msg::PoseStamped OffboardController::apply_geofence(
  const geometry_msgs::msg::PoseStamped::SharedPtr & pose)
{
  auto clamped = *pose;
  
  // 水平裁剪
  double distance = std::sqrt(clamped.pose.position.x * clamped.pose.position.x + 
                              clamped.pose.position.y * clamped.pose.position.y);
  if (distance > 50.0) {
    double scale = 50.0 / distance;
    clamped.pose.position.x *= scale;
    clamped.pose.position.y *= scale;
  }

  // 高度裁剪
  if (clamped.pose.position.z > 30.0) clamped.pose.position.z = 30.0;
  if (clamped.pose.position.z < 0.5) clamped.pose.position.z = 0.5;

  return clamped;
}

}  // namespace px4_flight
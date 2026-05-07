#include "px4_flight/external_pose_bridge.hpp"

namespace px4_flight
{

ExternalPoseBridge::ExternalPoseBridge(const rclcpp::NodeOptions & options)
: Node("external_pose_bridge", options)
{
  // 声明参数
  this->declare_parameter("point_lio_topic", "/Odometry");
  this->declare_parameter("px4_odom_topic", "/fmu/out/vehicle_odometry");
  this->declare_parameter("timeout_threshold", 0.5);        // 最小保底阈值
  this->declare_parameter("jump_threshold", 0.5);
  this->declare_parameter("max_acceptable_covariance", 1.0);
  this->declare_parameter("enable_smoothing", true);
  this->declare_parameter("smoothing_window_size", 5);

  point_lio_topic_ = this->get_parameter("point_lio_topic").as_string();
  px4_odom_topic_ = this->get_parameter("px4_odom_topic").as_string();
  timeout_threshold_ = this->get_parameter("timeout_threshold").as_double();
  jump_threshold_ = this->get_parameter("jump_threshold").as_double();
  max_acceptable_covariance_ = this->get_parameter("max_acceptable_covariance").as_double();
  enable_smoothing_ = this->get_parameter("enable_smoothing").as_bool();
  smoothing_window_size_ = this->get_parameter("smoothing_window_size").as_int();

  // 动态阈值初始化
  avg_interval_ = 0.1;        // 初始假设10Hz = 100ms
  max_interval_ = 0.1;
  dynamic_threshold_ = std::max(timeout_threshold_, THRESHOLD_MULTIPLIER * avg_interval_);

  // QoS配置 - BEST_EFFORT适配Fast-LIO2
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  point_lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    point_lio_topic_, qos_profile,
    std::bind(&ExternalPoseBridge::point_lio_callback, this, std::placeholders::_1));

  px4_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    px4_odom_topic_, 10,
    std::bind(&ExternalPoseBridge::px4_odometry_callback, this, std::placeholders::_1));

  quality_pub_ = this->create_publisher<std_msgs::msg::Float64>("odom_quality", 10);
  filtered_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("filtered_odom", 10);

  auto watchdog_period = std::chrono::milliseconds(100);  // 10Hz检查
  watchdog_timer_ = this->create_wall_timer(
    watchdog_period, std::bind(&ExternalPoseBridge::watchdog_callback, this));

  last_odom_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), 
    "External Pose Bridge initialized (dynamic threshold: %.3fs, max missed: %d)",
    dynamic_threshold_, MAX_ALLOWED_MISSED_FRAMES);
}

void ExternalPoseBridge::set_px4_interface(std::shared_ptr<PX4Interface> px4_interface)
{
  px4_interface_ = px4_interface;
}

void ExternalPoseBridge::point_lio_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto now = this->get_clock()->now();
  
  // 计算实际间隔并更新统计
  double interval = (now - last_odom_time_).seconds();
  if (!first_msg_) {
    update_interval_statistics(interval);
  }
  last_odom_time_ = now;
  first_msg_ = false;

  // 重置连续timeout计数
  if (consecutive_timeouts_ > 0) {
    RCLCPP_INFO(this->get_logger(), 
      "Odometry recovered after %d missed frames (interval: %.3fs, dynamic threshold: %.3fs)",
      consecutive_timeouts_, interval, dynamic_threshold_);
    consecutive_timeouts_ = 0;
  }

  // 调试：记录首次接收
  static bool logged_first = false;
  if (!logged_first) {
    RCLCPP_INFO(this->get_logger(), 
      "First odometry received! x=%.3f, y=%.3f, z=%.3f, interval=%.3fs",
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
      interval);
    logged_first = true;
  }

  // 质量检查
  if (!check_odometry_quality(msg)) {
    consecutive_errors_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Odometry quality check failed, consecutive errors: %d", consecutive_errors_);
    
    if (consecutive_errors_ > max_consecutive_errors_) {
      odom_valid_ = false;
      RCLCPP_ERROR(this->get_logger(), 
        "Odometry deemed invalid after %d consecutive errors", consecutive_errors_);
    }
    return;
  }

  consecutive_errors_ = 0;
  odom_valid_ = true;

  // 平滑处理
  nav_msgs::msg::Odometry::SharedPtr processed_msg = msg;
  if (enable_smoothing_) {
    processed_msg = smooth_odometry(msg);
  }

  last_valid_odom_ = processed_msg;

  // 发布质量信息
  std_msgs::msg::Float64 quality_msg;
  quality_msg.data = 1.0;
  quality_pub_->publish(quality_msg);
  filtered_odom_pub_->publish(*processed_msg);

  // 发送到PX4
  if (px4_interface_) {
    px4_interface_->send_visual_odometry(processed_msg);
  }
}

void ExternalPoseBridge::px4_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  (void)msg;
  // 可用于对比外部定位与EKF2融合结果
  // TODO: 实现EKF2输出监控
}

bool ExternalPoseBridge::check_odometry_quality(const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  if (!check_covariance(msg)) {
    return false;
  }

  if (detect_position_jump(msg)) {
    return false;
  }

  if (std::isnan(msg->pose.pose.position.x) || 
      std::isnan(msg->pose.pose.position.y) || 
      std::isnan(msg->pose.pose.position.z)) {
    RCLCPP_WARN(this->get_logger(), "NaN detected in odometry");
    return false;
  }

  return true;
}

bool ExternalPoseBridge::detect_position_jump(const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  if (!last_valid_odom_) {
    return false;
  }

  double dx = msg->pose.pose.position.x - last_valid_odom_->pose.pose.position.x;
  double dy = msg->pose.pose.position.y - last_valid_odom_->pose.pose.position.y;
  double dz = msg->pose.pose.position.z - last_valid_odom_->pose.pose.position.z;
  
  // 使用实际时间差
  double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double last_time = last_valid_odom_->header.stamp.sec + last_valid_odom_->header.stamp.nanosec * 1e-9;
  double dt = current_time - last_time;
  
  if (dt < 0.001) dt = 0.001;
  if (dt > 1.0) dt = avg_interval_;  // 使用平均间隔替代异常值

  double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
  double velocity = distance / dt;

  if (velocity > 10.0) {
    RCLCPP_WARN(this->get_logger(), 
      "Position jump detected: %.2f m in %.3f s (%.2f m/s)",
      distance, dt, velocity);
    return true;
  }

  return false;
}

bool ExternalPoseBridge::check_covariance(const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  double pos_cov = std::max({
    msg->pose.covariance[0], 
    msg->pose.covariance[7], 
    msg->pose.covariance[14]
  });
  
  if (pos_cov > max_acceptable_covariance_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Covariance too large: %.3f", pos_cov);
    return false;
  }

  return true;
}

nav_msgs::msg::Odometry::SharedPtr ExternalPoseBridge::smooth_odometry(
  const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  pose_history_.push_back(msg->pose.pose);
  if (pose_history_.size() > static_cast<size_t>(smoothing_window_size_)) {
    pose_history_.pop_front();
  }

  auto smoothed_msg = std::make_shared<nav_msgs::msg::Odometry>(*msg);
  
  if (pose_history_.size() >= 2) {
    double avg_x = 0, avg_y = 0, avg_z = 0;
    for (const auto & pose : pose_history_) {
      avg_x += pose.position.x;
      avg_y += pose.position.y;
      avg_z += pose.position.z;
    }
    smoothed_msg->pose.pose.position.x = avg_x / pose_history_.size();
    smoothed_msg->pose.pose.position.y = avg_y / pose_history_.size();
    smoothed_msg->pose.pose.position.z = avg_z / pose_history_.size();
  }

  return smoothed_msg;
}

// 【新增】更新间隔统计
void ExternalPoseBridge::update_interval_statistics(double interval)
{
  // 指数平滑更新平均间隔
  avg_interval_ = 0.9 * avg_interval_ + 0.1 * interval;
  
  // 更新最大间隔（带衰减）
  if (interval > max_interval_) {
    max_interval_ = interval;
  } else {
    max_interval_ = 0.95 * max_interval_ + 0.05 * interval;  // 缓慢衰减
  }
  
  // 计算动态阈值
  double calculated = THRESHOLD_MULTIPLIER * avg_interval_;
  dynamic_threshold_ = std::max(timeout_threshold_, calculated);
  dynamic_threshold_ = std::min(dynamic_threshold_, 2.0);  // 上限2秒
}

// 【新增】计算当前动态阈值
double ExternalPoseBridge::calculate_dynamic_threshold()
{
  return dynamic_threshold_;
}

void ExternalPoseBridge::watchdog_callback()
{
  auto now = this->get_clock()->now();
  double elapsed = (now - last_odom_time_).seconds();
  
  if (elapsed > dynamic_threshold_) {
    consecutive_timeouts_++;
    
    if (consecutive_timeouts_ >= MAX_ALLOWED_MISSED_FRAMES) {
      // 真正丢失，报警
      if (odom_valid_) {
        odom_valid_ = false;
        RCLCPP_ERROR(this->get_logger(), 
          "Odometry LOST! Last msg %.3fs ago (threshold: %.3fs), consecutive: %d/%d, avg interval: %.3fs",
          elapsed, dynamic_threshold_, consecutive_timeouts_, MAX_ALLOWED_MISSED_FRAMES, avg_interval_);
      }
      
      // 发送最后已知位置
      if (last_valid_odom_ && px4_interface_) {
        auto last_msg = std::make_shared<nav_msgs::msg::Odometry>(*last_valid_odom_);
        last_msg->header.stamp = now;
        last_msg->header.frame_id = "map";
        last_msg->twist.twist.linear.x = 0;
        last_msg->twist.twist.linear.y = 0;
        last_msg->twist.twist.linear.z = 0;
        px4_interface_->send_visual_odometry(last_msg);
      }
    } else {
      // 偶尔丢帧，仅警告
      RCLCPP_WARN(this->get_logger(), 
        "Odometry frame missed (%.3fs), consecutive: %d/%d, dynamic threshold: %.3fs",
        elapsed, consecutive_timeouts_, MAX_ALLOWED_MISSED_FRAMES, dynamic_threshold_);
    }
  } else {
    // 正常状态
    if (consecutive_timeouts_ > 0) {
      RCLCPP_INFO(this->get_logger(), 
        "Odometry recovered, missed frames cleared");
      consecutive_timeouts_ = 0;
    }
    
    // 定期打印状态（每10秒）
    static int cnt = 0;
    if (++cnt % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), 
        "Watchdog OK (avg interval: %.3fs, dynamic threshold: %.3fs, max interval: %.3fs)",
        avg_interval_, dynamic_threshold_, max_interval_);
    }
  }
}

}  // namespace px4_flight
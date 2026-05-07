#ifndef PX4_FLIGHT__EXTERNAL_POSE_BRIDGE_HPP_
#define PX4_FLIGHT__EXTERNAL_POSE_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "px4_flight/px4_interface.hpp"

#include <deque>
#include <mutex>

namespace px4_flight
{

class ExternalPoseBridge : public rclcpp::Node
{
public:
  explicit ExternalPoseBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ExternalPoseBridge() = default;

  void set_px4_interface(std::shared_ptr<PX4Interface> px4_interface);

private:
  void point_lio_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void px4_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool check_odometry_quality(const nav_msgs::msg::Odometry::SharedPtr & msg);
  bool detect_position_jump(const nav_msgs::msg::Odometry::SharedPtr & msg);
  bool check_covariance(const nav_msgs::msg::Odometry::SharedPtr & msg);

  nav_msgs::msg::Odometry::SharedPtr smooth_odometry(
    const nav_msgs::msg::Odometry::SharedPtr & msg);

  void watchdog_callback();

  // 动态阈值参数
  double calculate_dynamic_threshold();
  void update_interval_statistics(double interval);

  // 参数
  std::string point_lio_topic_;
  std::string px4_odom_topic_;
  double timeout_threshold_;           // 最小阈值（保底）
  double jump_threshold_;
  double max_acceptable_covariance_;
  bool enable_smoothing_;
  int smoothing_window_size_;
  
  // 动态阈值相关
  double avg_interval_;               // 历史平均间隔
  double max_interval_;               // 历史最大间隔
  double dynamic_threshold_;          // 动态计算的阈值
  static constexpr double THRESHOLD_MULTIPLIER = 3.0;  // 阈值 = 平均间隔 × 倍数
  static constexpr int MAX_ALLOWED_MISSED_FRAMES = 3;  // 允许连续丢帧数

  // 订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr point_lio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr px4_odom_sub_;

  // 发布者
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr quality_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;

  // PX4接口
  std::shared_ptr<PX4Interface> px4_interface_;

  // 状态
  nav_msgs::msg::Odometry::SharedPtr last_valid_odom_;
  rclcpp::Time last_odom_time_;
  std::deque<geometry_msgs::msg::Pose> pose_history_;
  bool odom_valid_{false};
  int consecutive_errors_{0};
  int consecutive_timeouts_{0};       // 连续timeout计数
  bool first_msg_{true};
  const int max_consecutive_errors_{5};

  // 定时器
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace px4_flight

#endif  // PX4_FLIGHT__EXTERNAL_POSE_BRIDGE_HPP_
  // 低通滤波状态
  double last_filtered_x_ = 0.0;
  double last_filtered_y_ = 0.0;
  double last_filtered_z_ = 0.0;

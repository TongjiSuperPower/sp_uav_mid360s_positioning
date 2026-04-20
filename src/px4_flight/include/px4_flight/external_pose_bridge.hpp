#ifndef PX4_FLIGHT__EXTERNAL_POSE_BRIDGE_HPP_
#define PX4_FLIGHT__EXTERNAL_POSE_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "px4_flight/px4_interface.hpp"
#include <deque>

namespace px4_flight
{

/**
 * @brief 外部定位桥接节点
 * 接收Point-LIO输出，处理质量检测，转发至PX4
 */
class ExternalPoseBridge : public rclcpp::Node
{
public:
  explicit ExternalPoseBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ExternalPoseBridge() = default;

  void set_px4_interface(std::shared_ptr<PX4Interface> px4_interface);

private:
  // 订阅回调
  void point_lio_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void px4_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // 质量检测
  bool check_odometry_quality(const nav_msgs::msg::Odometry::SharedPtr & msg);
  bool detect_position_jump(const nav_msgs::msg::Odometry::SharedPtr & msg);
  bool check_covariance(const nav_msgs::msg::Odometry::SharedPtr & msg);

  // 数据平滑
  nav_msgs::msg::Odometry::SharedPtr smooth_odometry(
    const nav_msgs::msg::Odometry::SharedPtr & msg);

  // 定时器
  void watchdog_callback();

  // 参数
  std::string point_lio_topic_;
  std::string px4_odom_topic_;
  double timeout_threshold_;
  double jump_threshold_;
  double max_acceptable_covariance_;
  bool enable_smoothing_;
  int smoothing_window_size_;

  // 订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr point_lio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr px4_odom_sub_;

  // 发布者（用于调试）
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr quality_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // PX4接口
  std::shared_ptr<PX4Interface> px4_interface_;

  // 状态
  nav_msgs::msg::Odometry::SharedPtr last_valid_odom_;
  rclcpp::Time last_odom_time_;
  std::deque<geometry_msgs::msg::Pose> pose_history_;
  bool odom_valid_{false};
  int consecutive_errors_{0};
  const int max_consecutive_errors_{5};
};

}  // namespace px4_flight

#endif  // PX4_FLIGHT__EXTERNAL_POSE_BRIDGE_HPP_
#ifndef QR_LANDING__QR_VARIANCE_MONITOR_HPP_
#define QR_LANDING__QR_VARIANCE_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <deque>
#include <cmath>

namespace qr_landing
{

struct Sample {
  double timestamp;
  double x;
  double y;
  double z;
  double yaw;  // unwrapped yaw (rad)
};

class QRVarianceMonitorNode : public rclcpp::Node
{
public:
  explicit QRVarianceMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void printStats();
  
  static double extractStableYawRad(double qx, double qy, double qz, double qw);
  static double normDeg(double rad);
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::deque<Sample> buffer_;
  double window_seconds_;
  
  // yaw unwrap state
  double prev_yaw_raw_ = 0.0;
  double yaw_offset_ = 0.0;
  bool yaw_init_ = false;
};

} // namespace qr_landing

#endif // QR_LANDING__QR_VARIANCE_MONITOR_HPP_
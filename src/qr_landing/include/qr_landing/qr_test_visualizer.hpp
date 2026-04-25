#ifndef QR_LANDING__QR_TEST_VISUALIZER_HPP_
#define QR_LANDING__QR_TEST_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <iomanip>
#include <sstream>

namespace qr_landing
{

class QRTestVisualizerNode : public rclcpp::Node
{
public:
  explicit QRTestVisualizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void debugImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void qrRelativeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void printTimerCallback();
  
  void drawOverlay(cv::Mat & img);
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qr_relative_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr test_image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;   // <-- 新增：xyz
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;     // <-- 新增：rpy(deg)
  
  rclcpp::TimerBase::SharedPtr print_timer_;
  
  geometry_msgs::msg::PoseStamped latest_qr_relative_;
  bool qr_valid_ = false;
  rclcpp::Time last_qr_time_;
  std::mutex data_mutex_;
  cv::Mat latest_img_;
  bool img_valid_ = false;
};

} // namespace qr_landing

#endif // QR_LANDING__QR_TEST_VISUALIZER_HPP_
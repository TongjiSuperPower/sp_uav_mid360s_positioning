#ifndef QR_LANDING__COORDINATE_TRANSFORMER_HPP_
#define QR_LANDING__COORDINATE_TRANSFORMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace qr_landing
{

class CoordinateTransformerNode : public rclcpp::Node
{
public:
  explicit CoordinateTransformerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void qrPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qr_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr qr_relative_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr qr_global_pub_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  std::string target_frame_;   // base_link
  std::string global_frame_;   // map (optional)
};

} // namespace qr_landing

#endif // QR_LANDING__COORDINATE_TRANSFORMER_HPP_
#include "qr_landing/coordinate_transformer.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace qr_landing
{

CoordinateTransformerNode::CoordinateTransformerNode(const rclcpp::NodeOptions & options)
: Node("qr_coordinate_transformer", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  this->declare_parameter("target_frame", "base_link");
  this->declare_parameter("global_frame", "map");
  
  target_frame_ = this->get_parameter("target_frame").as_string();
  global_frame_ = this->get_parameter("global_frame").as_string();
  
  qr_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/qr_landing/qr_pose_raw", 10,
    std::bind(&CoordinateTransformerNode::qrPoseCallback, this, std::placeholders::_1));
  
  qr_relative_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/qr_landing/qr_relative", 10);
  qr_global_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/qr_landing/qr_pose_map", 10);
  
  RCLCPP_INFO(this->get_logger(),
    "CoordinateTransformer: target=%s, global=%s", target_frame_.c_str(), global_frame_.c_str());
}

void CoordinateTransformerNode::qrPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  try {
    geometry_msgs::msg::TransformStamped cam_to_base = tf_buffer_.lookupTransform(
      target_frame_, msg->header.frame_id,
      msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    
    geometry_msgs::msg::PoseStamped qr_in_base;
    tf2::doTransform(*msg, qr_in_base, cam_to_base);
    qr_in_base.header.frame_id = target_frame_;
    
    qr_relative_pub_->publish(qr_in_base);
    
    try {
      geometry_msgs::msg::TransformStamped base_to_map = tf_buffer_.lookupTransform(
        global_frame_, target_frame_,
        msg->header.stamp, rclcpp::Duration::from_seconds(0.05));
      
      geometry_msgs::msg::PoseStamped qr_in_map;
      tf2::doTransform(qr_in_base, qr_in_map, base_to_map);
      qr_in_map.header.frame_id = global_frame_;
      qr_global_pub_->publish(qr_in_map);
      
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "Global TF unavailable: %s", ex.what());
    }
    
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "TF camera->base failed: %s", ex.what());
  }
}

} // namespace qr_landing

// --- 传统 main 函数入口 ---
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qr_landing::CoordinateTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
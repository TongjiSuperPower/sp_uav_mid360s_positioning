#include "qr_landing/qr_test_visualizer.hpp"
#include <cmath>

namespace qr_landing
{

// 稳定偏航提取：替代 getRPY，避免下视相机万向节锁
inline double extractStableYaw(double qx, double qy, double qz, double qw)
{
  double vx = 1.0 - 2.0 * (qy * qy + qz * qz);
  double vy = 2.0 * (qx * qy + qw * qz);
  double yaw = std::atan2(vy, vx) - (M_PI / 2.0);  // 减去 90° 补偿 ArUco 坐标系偏移
  return yaw;
}

QRTestVisualizerNode::QRTestVisualizerNode(const rclcpp::NodeOptions & options)
: Node("qr_test_visualizer", options)
{
  this->declare_parameter("print_rate", 10.0);
  this->declare_parameter("image_topic", "/qr_landing/debug_image");
  this->declare_parameter("qr_relative_topic", "/qr_landing/qr_relative");
  this->declare_parameter("output_topic", "/qr_landing/test_visualization");
  
  std::string image_topic = this->get_parameter("image_topic").as_string();
  std::string qr_topic = this->get_parameter("qr_relative_topic").as_string();
  std::string out_topic = this->get_parameter("output_topic").as_string();
  double print_rate = this->get_parameter("print_rate").as_double();
  
  debug_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(),
    std::bind(&QRTestVisualizerNode::debugImageCallback, this, std::placeholders::_1));
    
  qr_relative_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    qr_topic, 10,
    std::bind(&QRTestVisualizerNode::qrRelativeCallback, this, std::placeholders::_1));
  
  test_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_topic, 10);
  position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/qr_landing/qr_position", 10);      // <-- 新增：xyz [m]
  euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/qr_landing/qr_euler", 10);          // <-- 新增：roll/pitch/yaw [deg]
  
  print_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / print_rate)),
    std::bind(&QRTestVisualizerNode::printTimerCallback, this));
  
  RCLCPP_INFO(this->get_logger(), 
    "QRTestVisualizer ready. image=%s, qr=%s, print=%.1fHz", 
    image_topic.c_str(), qr_topic.c_str(), print_rate);
  RCLCPP_INFO(this->get_logger(), 
    "Publishing: /qr_landing/qr_position (PointStamped), /qr_landing/qr_euler (Vector3Stamped, deg)");
}

void QRTestVisualizerNode::debugImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_img_ = cv_ptr->image.clone();
    img_valid_ = true;
    
    drawOverlay(latest_img_);
    
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", latest_img_).toImageMsg();
    test_image_pub_->publish(*out_msg);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void QRTestVisualizerNode::qrRelativeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_qr_relative_ = *msg;
  qr_valid_ = true;
  last_qr_time_ = this->now();
  
  // --- 实时发布 xyz 位置 ---
  geometry_msgs::msg::PointStamped pos_msg;
  pos_msg.header = msg->header;
  pos_msg.point.x = msg->pose.position.x;
  pos_msg.point.y = msg->pose.position.y;
  pos_msg.point.z = msg->pose.position.z;
  position_pub_->publish(pos_msg);
  
  // --- 实时发布欧拉角 (deg) ---
  tf2::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);
  
  // roll/pitch 使用标准 getRPY（在 base_link 下，水平放置的码 roll≈0, pitch≈0）
  double roll, pitch, yaw_rpy;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_rpy);
  
  // yaw 使用稳定提取（避免万向节锁跳变）
  double yaw_stable = extractStableYaw(q.x(), q.y(), q.z(), q.w());
  
  // 全部归一化到 [-180, 180] 并转度
  auto norm_deg = [](double rad) {
    double deg = rad * 180.0 / M_PI;
    while (deg > 180.0)  deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
  };
  
  geometry_msgs::msg::Vector3Stamped euler_msg;
  euler_msg.header = msg->header;
  euler_msg.vector.x = norm_deg(roll);        // roll [deg]
  euler_msg.vector.y = norm_deg(pitch);       // pitch [deg]
  euler_msg.vector.z = norm_deg(yaw_stable);  // yaw [deg]（稳定版）
  euler_pub_->publish(euler_msg);
}

void QRTestVisualizerNode::drawOverlay(cv::Mat & img)
{
  if (!qr_valid_) return;
  if ((this->now() - last_qr_time_).seconds() > 0.5) {
    qr_valid_ = false;
    return;
  }
  
  double x = latest_qr_relative_.pose.position.x;
  double y = latest_qr_relative_.pose.position.y;
  double z = latest_qr_relative_.pose.position.z;
  double height = -z;  // 正值高度（离地面距离）
  
  tf2::Quaternion q(
    latest_qr_relative_.pose.orientation.x,
    latest_qr_relative_.pose.orientation.y,
    latest_qr_relative_.pose.orientation.z,
    latest_qr_relative_.pose.orientation.w);
  double yaw = extractStableYaw(q.x(), q.y(), q.z(), q.w());
  double yaw_deg = yaw * 180.0 / M_PI;
  while (yaw_deg > 180.0) yaw_deg -= 360.0;
  while (yaw_deg < -180.0) yaw_deg += 360.0;
  
  // 右上角：显示 X Y H(高度) Yaw
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "X:" << x << " Y:" << y << " H:" << height << " Yaw:" << yaw_deg;
  
  std::string text = oss.str();
  int font = cv::FONT_HERSHEY_SIMPLEX;
  double font_scale = 0.55;
  int thickness = 2;
  cv::Size ts = cv::getTextSize(text, font, font_scale, thickness, nullptr);
  
  cv::Point origin(img.cols - ts.width - 15, 30);
  cv::rectangle(img,
    cv::Point(origin.x - 5, origin.y - ts.height - 5),
    cv::Point(origin.x + ts.width + 5, origin.y + 5),
    cv::Scalar(0, 0, 0), -1);
  cv::putText(img, text, origin, font, font_scale, cv::Scalar(0, 255, 0), thickness);
  
  std::string label = "QR LOCKED [base_link]";
  cv::Size ts2 = cv::getTextSize(label, font, 0.45, 1, nullptr);
  cv::rectangle(img,
    cv::Point(5, 5),
    cv::Point(5 + ts2.width + 10, 5 + ts2.height + 10),
    cv::Scalar(0, 0, 0), -1);
  cv::putText(img, label, cv::Point(10, 20), font, 0.45, cv::Scalar(0, 255, 0), 1);
}

void QRTestVisualizerNode::printTimerCallback()
{
  if (!qr_valid_ || (this->now() - last_qr_time_).seconds() > 0.5) {
    qr_valid_ = false;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[TEST] No QR detected or timeout");
    return;
  }
  
  double x = latest_qr_relative_.pose.position.x;
  double y = latest_qr_relative_.pose.position.y;
  double z = latest_qr_relative_.pose.position.z;
  double height = -z;
  
  tf2::Quaternion q(
    latest_qr_relative_.pose.orientation.x,
    latest_qr_relative_.pose.orientation.y,
    latest_qr_relative_.pose.orientation.z,
    latest_qr_relative_.pose.orientation.w);
  double yaw = extractStableYaw(q.x(), q.y(), q.z(), q.w());
  double yaw_deg = yaw * 180.0 / M_PI;
  while (yaw_deg > 180.0) yaw_deg -= 360.0;
  while (yaw_deg < -180.0) yaw_deg += 360.0;
  
  RCLCPP_INFO(this->get_logger(), 
    "[REL] x=%+.3f y=%+.3f h=%+.3f | yaw=%+7.2fdeg | frame=%s",
    x, y, height, yaw_deg,
    latest_qr_relative_.header.frame_id.c_str());
}

} // namespace qr_landing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qr_landing::QRTestVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
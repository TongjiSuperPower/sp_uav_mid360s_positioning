#include "qr_landing/qr_variance_monitor.hpp"

namespace qr_landing
{

QRVarianceMonitorNode::QRVarianceMonitorNode(const rclcpp::NodeOptions & options)
: Node("qr_variance_monitor", options)
{
  this->declare_parameter("window_seconds", 10.0);
  this->declare_parameter("topic", "/qr_landing/qr_relative");
  
  window_seconds_ = this->get_parameter("window_seconds").as_double();
  std::string topic = this->get_parameter("topic").as_string();
  
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic, 10,
    std::bind(&QRVarianceMonitorNode::poseCallback, this, std::placeholders::_1));
  
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(window_seconds_ * 1000.0)),
    std::bind(&QRVarianceMonitorNode::printStats, this));
  
  RCLCPP_INFO(this->get_logger(),
    "QRVarianceMonitor started. window=%.1fs, topic=%s", window_seconds_, topic.c_str());
}

double QRVarianceMonitorNode::extractStableYawRad(double qx, double qy, double qz, double qw)
{
  double vx = 1.0 - 2.0 * (qy * qy + qz * qz);
  double vy = 2.0 * (qx * qy + qw * qz);
  double yaw = std::atan2(vy, vx) - (M_PI / 2.0);
  while (yaw > M_PI)  yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

double QRVarianceMonitorNode::normDeg(double rad)
{
  double deg = rad * 180.0 / M_PI;
  while (deg > 180.0)  deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

void QRVarianceMonitorNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  
  double yaw_raw = extractStableYawRad(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);
  
  // unwrap yaw to avoid ±180 jump
  if (!yaw_init_) {
    yaw_offset_ = 0.0;
    yaw_init_ = true;
  } else {
    double delta = yaw_raw - prev_yaw_raw_;
    if (delta > M_PI)       yaw_offset_ -= 2.0 * M_PI;
    else if (delta < -M_PI)  yaw_offset_ += 2.0 * M_PI;
  }
  prev_yaw_raw_ = yaw_raw;
  double yaw_uw = yaw_raw + yaw_offset_;
  
  double now = this->get_clock()->now().seconds();
  buffer_.push_back({now, x, y, z, yaw_uw});
  
  // remove old samples outside window
  while (!buffer_.empty() && (now - buffer_.front().timestamp) > window_seconds_) {
    buffer_.pop_front();
  }
}

void QRVarianceMonitorNode::printStats()
{
  if (buffer_.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "[VAR] 数据不足(%zu < 2)，跳过本次统计", buffer_.size());
    return;
  }
  
  auto compute = [](const std::deque<Sample> & buf, double Sample::* field) {
    double min_v = 1e308, max_v = -1e308, sum = 0.0;
    for (const auto & s : buf) {
      double v = s.*field;
      if (v < min_v) min_v = v;
      if (v > max_v) max_v = v;
      sum += v;
    }
    double mean = sum / static_cast<double>(buf.size());
    double sq_sum = 0.0;
    for (const auto & s : buf) {
      double diff = s.*field - mean;
      sq_sum += diff * diff;
    }
    double variance = sq_sum / static_cast<double>(buf.size() - 1);  // 样本方差
    return std::make_tuple(min_v, max_v, mean, variance);
  };
  
  auto [xmin, xmax, xmean, xvar] = compute(buffer_, &Sample::x);
  auto [ymin, ymax, ymean, yvar] = compute(buffer_, &Sample::y);
  auto [zmin, zmax, zmean, zvar] = compute(buffer_, &Sample::z);
  auto [yawmin, yawmax, yawmean, yawvar] = compute(buffer_, &Sample::yaw);
  
  // normalize yaw mean back to [-180,180] for display
  double yaw_mean_deg = normDeg(yawmean);
  double yaw_min_deg  = normDeg(yawmin);
  double yaw_max_deg  = normDeg(yawmax);
  double yaw_var_deg2 = yawvar * 180.0 * 180.0 / (M_PI * M_PI);  // rad² → deg²
  
  RCLCPP_INFO(this->get_logger(),
    "\n"
    "╔══════════════════════════════════════════════════════════════╗\n"
    "║          QR Variance Monitor  (%.1fs 窗口, N=%3zu)            ║\n"
    "╠══════════════════════════════════════════════════════════════╣\n"
    "║  X(m)    min=%+.4f  max=%+.4f  mean=%+.4f  var=%.6f        ║\n"
    "║  Y(m)    min=%+.4f  max=%+.4f  mean=%+.4f  var=%.6f        ║\n"
    "║  Z(m)    min=%+.4f  max=%+.4f  mean=%+.4f  var=%.6f        ║\n"
    "║  Yaw(°)  min=%+7.2f max=%+7.2f mean=%+7.2f var=%7.3f       ║\n"
    "╚══════════════════════════════════════════════════════════════╝",
    window_seconds_, buffer_.size(),
    xmin, xmax, xmean, xvar,
    ymin, ymax, ymean, yvar,
    zmin, zmax, zmean, zvar,
    yaw_min_deg, yaw_max_deg, yaw_mean_deg, yaw_var_deg2
  );
}

} // namespace qr_landing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qr_landing::QRVarianceMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
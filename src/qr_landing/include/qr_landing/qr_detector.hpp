#ifndef QR_LANDING__QR_DETECTOR_HPP_
#define QR_LANDING__QR_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <map>
#include <vector>

namespace qr_landing
{

struct MatrixCell {
  int id;
  int row;
  int col;
};

class QRDetectorNode : public rclcpp::Node
{
public:
  explicit QRDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  float getDepthAtCenter(const std::vector<cv::Point2f> & corners);
  
  bool buildMatrixMap();
  
  // 多码位置融合 + 单码/多码姿态 EMA
  geometry_msgs::msg::PoseStamped fuseMatrixPose(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<int> & ids,
    const std::vector<cv::Vec3d> & tvecs,
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<std::vector<cv::Point2f>> & corners);
  
  void drawDebug(
    cv::Mat & img,
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<cv::Vec3d> & tvecs,
    const geometry_msgs::msg::PoseStamped & fused_pose);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr qr_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  
  // --- 三个 debug 话题 ---
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr id4_yaw_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr id4_yaw_ema_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr avg_yaw_ema_pub_;
  
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool camera_ready_ = false;
  
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  std::mutex depth_mutex_;
  
  double marker_size_;
  bool publish_debug_;
  bool matrix_mode_;
  double matrix_cell_pitch_;
  double depth_pnp_max_diff_ratio_;
  std::vector<long int> matrix_ids_;
  std::map<int, MatrixCell> matrix_map_;
  
  // --- 每个 ID 独立的四元数 EMA 状态 ---
  std::map<int, tf2::Quaternion> ema_quat_map_;
  std::map<int, bool> ema_quat_init_;
  double ema_alpha_yaw_;
};

} // namespace qr_landing

#endif // QR_LANDING__QR_DETECTOR_HPP_
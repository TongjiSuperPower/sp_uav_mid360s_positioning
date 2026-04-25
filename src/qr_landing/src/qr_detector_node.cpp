#include "qr_landing/qr_detector.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace qr_landing
{

QRDetectorNode::QRDetectorNode(const rclcpp::NodeOptions & options)
: Node("qr_detector", options)
{
  this->declare_parameter("marker_size", 0.10);           // 10cm 边长
  this->declare_parameter("aruco_dict", "DICT_4X4_50");
  this->declare_parameter("camera_topic", "/camera/camera/color/image_raw");
  this->declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  this->declare_parameter("camera_info_topic", "/camera/camera/color/camera_info");
  this->declare_parameter("publish_debug_image", true);
  this->declare_parameter("matrix_mode", true);
  this->declare_parameter("matrix_ids", std::vector<long int>{0,1,2,3,4,5,6,7,8});
  this->declare_parameter("matrix_cell_pitch", 0.12);     // 12cm 中心距
  this->declare_parameter("depth_pnp_max_diff_ratio", 0.30);
  
  marker_size_ = this->get_parameter("marker_size").as_double();
  publish_debug_ = this->get_parameter("publish_debug_image").as_bool();
  matrix_mode_ = this->get_parameter("matrix_mode").as_bool();
  matrix_cell_pitch_ = this->get_parameter("matrix_cell_pitch").as_double();
  depth_pnp_max_diff_ratio_ = this->get_parameter("depth_pnp_max_diff_ratio").as_double();
  matrix_ids_ = this->get_parameter("matrix_ids").as_integer_array();
  
  if (matrix_mode_ && !buildMatrixMap()) {
    RCLCPP_ERROR(this->get_logger(), "Matrix ID list must contain exactly 9 IDs!");
    matrix_mode_ = false;
  }
  
  std::string dict_name = this->get_parameter("aruco_dict").as_string();
  if (dict_name == "DICT_4X4_50") {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  } else if (dict_name == "DICT_5X5_50") {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  } else {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  }
  
  detector_params_ = cv::aruco::DetectorParameters::create();
  
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    this->get_parameter("camera_topic").as_string(),
    rclcpp::SensorDataQoS(),
    std::bind(&QRDetectorNode::imageCallback, this, std::placeholders::_1));
    
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    this->get_parameter("depth_topic").as_string(),
    rclcpp::SensorDataQoS(),
    std::bind(&QRDetectorNode::depthCallback, this, std::placeholders::_1));
    
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    this->get_parameter("camera_info_topic").as_string(),
    10,
    std::bind(&QRDetectorNode::cameraInfoCallback, this, std::placeholders::_1));
  
  qr_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/qr_landing/qr_pose_raw", 10);
  debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/qr_landing/debug_image", 10);
  
  RCLCPP_INFO(this->get_logger(),
    "QRDetector ready. marker=%.3fm, matrix=%s, pitch=%.3fm",
    marker_size_, matrix_mode_ ? "ON" : "OFF", matrix_cell_pitch_);
}

bool QRDetectorNode::buildMatrixMap()
{
  if (matrix_ids_.size() != 9) return false;
  int idx = 0;
  for (int r = -1; r <= 1; ++r) {
    for (int c = -1; c <= 1; ++c) {
      int id = static_cast<int>(matrix_ids_[idx]);
      matrix_map_[id] = {id, r, c};
      ++idx;
    }
  }
  return true;
}

void QRDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!camera_ready_) {
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
    camera_ready_ = true;
    RCLCPP_INFO(this->get_logger(),
      "Camera intrinsics received. fx=%.2f, fy=%.2f",
      camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(1,1));
  }
}

void QRDetectorNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  latest_depth_ = msg;
}

float QRDetectorNode::getDepthAtCenter(const std::vector<cv::Point2f> & corners)
{
  if (!latest_depth_) return -1.0f;
  
  cv::Point2f center(0.0f, 0.0f);
  for (const auto & p : corners) center += p;
  center *= (1.0f / corners.size());
  
  int u = static_cast<int>(std::round(center.x));
  int v = static_cast<int>(std::round(center.y));
  
  cv_bridge::CvImageConstPtr cv_depth;
  try {
    cv_depth = cv_bridge::toCvShare(latest_depth_, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (...) {
    return -1.0f;
  }
  
  if (u < 0 || u >= cv_depth->image.cols || v < 0 || v >= cv_depth->image.rows)
    return -1.0f;
  
  uint16_t depth_mm = cv_depth->image.at<uint16_t>(v, u);
  if (depth_mm == 0) return -1.0f;
  
  return static_cast<float>(depth_mm) / 1000.0f;
}

geometry_msgs::msg::PoseStamped QRDetectorNode::fuseMatrixPose(
  const builtin_interfaces::msg::Time & stamp,
  const std::vector<int> & ids,
  const std::vector<cv::Vec3d> & tvecs,
  const std::vector<cv::Vec3d> & rvecs,
  const std::vector<std::vector<cv::Point2f>> & corners)
{
  std::vector<cv::Vec3d> center_tvecs;
  std::vector<cv::Vec3d> center_rvecs;
  std::vector<double> weights;
  
  for (size_t i = 0; i < ids.size(); ++i) {
    auto it = matrix_map_.find(ids[i]);
    if (it == matrix_map_.end()) continue;
    
    int row = it->second.row;
    int col = it->second.col;
    
    float depth_m = getDepthAtCenter(corners[i]);
    float pnp_z = static_cast<float>(tvecs[i][2]);
    
    float use_z = pnp_z;
    if (depth_m >= 0.2f && std::abs(depth_m - pnp_z) <= static_cast<float>(depth_pnp_max_diff_ratio_ * pnp_z)) {
      use_z = depth_m;
    } else if (depth_m >= 0.2f) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "ID%d depth inconsistent (%.2f vs %.2f), using PnP", ids[i], depth_m, pnp_z);
    }
    
    double offset_x = static_cast<double>(col) * matrix_cell_pitch_;
    double offset_y = static_cast<double>(row) * matrix_cell_pitch_;
    
    cv::Vec3d center_t;
    center_t[0] = tvecs[i][0] - offset_x;
    center_t[1] = tvecs[i][1] - offset_y;
    center_t[2] = use_z;
    
    center_tvecs.push_back(center_t);
    center_rvecs.push_back(rvecs[i]);
    weights.push_back(1.0);
  }
  
  if (center_tvecs.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  
  cv::Vec3d mean_t(0,0,0);
  for (const auto & t : center_tvecs) mean_t += t;
  mean_t *= (1.0 / center_tvecs.size());
  
  std::vector<cv::Vec3d> filtered_t;
  std::vector<cv::Vec3d> filtered_r;
  for (size_t i = 0; i < center_tvecs.size(); ++i) {
    double err = cv::norm(center_tvecs[i] - mean_t);
    if (err < 0.20 * cv::norm(mean_t) + 0.05) {
      filtered_t.push_back(center_tvecs[i]);
      filtered_r.push_back(center_rvecs[i]);
    } else {
      RCLCPP_WARN(this->get_logger(), "Outlier ID? rejected, err=%.3f", err);
    }
  }
  
  if (filtered_t.empty()) filtered_t = center_tvecs;
  
  cv::Vec3d fused_t(0,0,0);
  for (const auto & t : filtered_t) fused_t += t;
  fused_t *= (1.0 / filtered_t.size());
  
  cv::Vec3d fused_r(0,0,0);
  for (const auto & r : filtered_r) fused_r += r;
  fused_r *= (1.0 / filtered_r.size());
  
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = "camera_color_optical_frame";
  
  pose_msg.pose.position.x = fused_t[0];
  pose_msg.pose.position.y = fused_t[1];
  pose_msg.pose.position.z = fused_t[2];
  
  cv::Mat rot_mat;
  cv::Rodrigues(fused_r, rot_mat);
  tf2::Matrix3x3 tf2_rot(
    rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
    rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
    rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2)
  );
  tf2::Quaternion tf2_quat;
  tf2_rot.getRotation(tf2_quat);
  
  pose_msg.pose.orientation.x = tf2_quat.x();
  pose_msg.pose.orientation.y = tf2_quat.y();
  pose_msg.pose.orientation.z = tf2_quat.z();
  pose_msg.pose.orientation.w = tf2_quat.w();
  
  RCLCPP_DEBUG(this->get_logger(),
    "Fused %zu/%zu markers -> center=[%.3f, %.3f, %.3f]",
    filtered_t.size(), ids.size(), fused_t[0], fused_t[1], fused_t[2]);
  
  return pose_msg;
}

void QRDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!camera_ready_) return;
  
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids, detector_params_);
    
    if (marker_ids.empty()) {
      return;
    }
    
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
      marker_corners, marker_size_,
      camera_matrix_, dist_coeffs_,
      rvecs, tvecs);
    
    geometry_msgs::msg::PoseStamped fused_pose;
    
    if (matrix_mode_ && marker_ids.size() >= 1) {
      fused_pose = fuseMatrixPose(msg->header.stamp, marker_ids, tvecs, rvecs, marker_corners);
      if (fused_pose.header.frame_id.empty()) {
        return;
      }
    } else {
      float depth_m = getDepthAtCenter(marker_corners[0]);
      float pnp_z = static_cast<float>(tvecs[0][2]);
      if (depth_m < 0.2f || std::abs(depth_m - pnp_z) > static_cast<float>(depth_pnp_max_diff_ratio_ * pnp_z)) {
        depth_m = pnp_z;
      }
      fused_pose.header.stamp = msg->header.stamp;
      fused_pose.header.frame_id = "camera_color_optical_frame";
      fused_pose.pose.position.x = tvecs[0][0];
      fused_pose.pose.position.y = tvecs[0][1];
      fused_pose.pose.position.z = depth_m;
      
      cv::Mat rot_mat;
      cv::Rodrigues(rvecs[0], rot_mat);
      tf2::Matrix3x3 tf2_rot(
        rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
        rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
        rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2)
      );
      tf2::Quaternion tf2_quat;
      tf2_rot.getRotation(tf2_quat);
      fused_pose.pose.orientation.x = tf2_quat.x();
      fused_pose.pose.orientation.y = tf2_quat.y();
      fused_pose.pose.orientation.z = tf2_quat.z();
      fused_pose.pose.orientation.w = tf2_quat.w();
    }
    
    qr_pose_pub_->publish(fused_pose);
    
    if (publish_debug_) {
      drawDebug(cv_ptr->image, marker_corners, marker_ids, rvecs, tvecs, fused_pose);
      auto debug_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "bgr8", cv_ptr->image).toImageMsg();
      debug_msg->header = msg->header;
      debug_image_pub_->publish(*debug_msg);
    }
    
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void QRDetectorNode::drawDebug(
  cv::Mat & img,
  const std::vector<std::vector<cv::Point2f>> & corners,
  const std::vector<int> & ids,
  const std::vector<cv::Vec3d> & rvecs,
  const std::vector<cv::Vec3d> & tvecs,
  const geometry_msgs::msg::PoseStamped & fused_pose)
{
  cv::aruco::drawDetectedMarkers(img, corners, ids);
  
  for (size_t i = 0; i < ids.size(); ++i) {
    cv::drawFrameAxes(img, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_size_ * 0.5f);
  }
  
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "Fused Z:" << fused_pose.pose.position.z << "m  N:" << ids.size();
  cv::putText(img, oss.str(), cv::Point(10, 30),
    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
}

} // namespace qr_landing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qr_landing::QRDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
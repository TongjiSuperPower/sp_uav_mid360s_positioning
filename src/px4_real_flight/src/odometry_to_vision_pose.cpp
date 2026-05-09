#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <algorithm>

class OdometryToVisionPose : public rclcpp::Node
{
public:
    OdometryToVisionPose() : Node("odometry_to_vision_pose"), 
                              first_msg_(true), 
                              fusion_confirmed_(false),
                              offset_x_(0.0), offset_y_(0.0), offset_z_(0.0)
    {
        // 参数声明
        this->declare_parameter<std::string>("input_topic", "/Odometry");
        this->declare_parameter<std::string>("output_topic", "/mavros/vision_pose/pose");
        this->declare_parameter<std::string>("output_cov_topic", "/mavros/vision_pose/pose_cov");
        this->declare_parameter<std::string>("frame_id", "odom");
        this->declare_parameter<bool>("align_origin", true);       // 第一帧作为原点
        this->declare_parameter<bool>("publish_covariance", true); // 同时发布带协方差版本
        this->declare_parameter<double>("covariance_xy", 1.0);     // 启动大协方差，帮助 EKF2 克服内部偏移
        this->declare_parameter<double>("covariance_z", 0.1);

        std::string input_topic      = this->get_parameter("input_topic").as_string();
        std::string output_topic     = this->get_parameter("output_topic").as_string();
        std::string output_cov_topic = this->get_parameter("output_cov_topic").as_string();
        frame_id_        = this->get_parameter("frame_id").as_string();
        align_origin_    = this->get_parameter("align_origin").as_bool();
        publish_cov_     = this->get_parameter("publish_covariance").as_bool();
        cov_xy_ = this->get_parameter("covariance_xy").as_double();
        cov_z_  = this->get_parameter("covariance_z").as_double();

        // 订阅 FAST-LIO2
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic, 10,
            std::bind(&OdometryToVisionPose::odomCallback, this, std::placeholders::_1));

        // 发布到 MAVROS2 (PoseStamped，必支持)
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            output_topic, 10);

        // 可选发布带协方差版本 (PoseWithCovarianceStamped，若 MAVROS2 订阅 pose_cov 则生效)
        if (publish_cov_) {
            pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                output_cov_topic, 10);
        }

        // 监听 EKF2 融合状态
        status_sub_ = this->create_subscription<mavros_msgs::msg::EstimatorStatus>(
            "/mavros/estimator_status", 10,
            std::bind(&OdometryToVisionPose::statusCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "Bridge active: [%s] -> [%s] + [%s] | align_origin=%s | init_cov_xy=%.3f",
            input_topic.c_str(), output_topic.c_str(), output_cov_topic.c_str(),
            align_origin_ ? "true" : "false", cov_xy_);
    }

private:
    void statusCallback(const mavros_msgs::msg::EstimatorStatus::SharedPtr msg)
    {
        if (!fusion_confirmed_ && msg->pos_horiz_rel_status_flag) {
            fusion_confirmed_ = true;
            RCLCPP_WARN(this->get_logger(), 
                ">>> EKF2 horizontal fusion CONFIRMED! You can switch to Position mode now.");
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 第一帧对齐：将 FAST-LIO2 当前位置设为原点 (0,0,0)
        if (align_origin_ && first_msg_) {
            offset_x_ = msg->pose.pose.position.x;
            offset_y_ = msg->pose.pose.position.y;
            offset_z_ = msg->pose.pose.position.z;
            first_msg_ = false;
            
            RCLCPP_INFO(this->get_logger(),
                "Origin aligned: offset=[%.3f, %.3f, %.3f]",
                offset_x_, offset_y_, offset_z_);
        }

        // 1. 发布 PoseStamped (ENU，MAVROS2 内部自动转 NED)
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp    = msg->header.stamp;
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose.position.x = msg->pose.pose.position.x - offset_x_;
        pose_msg.pose.position.y = msg->pose.pose.position.y - offset_y_;
        pose_msg.pose.position.z = msg->pose.pose.position.z - offset_z_;
        pose_msg.pose.orientation = msg->pose.pose.orientation;
        pose_pub_->publish(pose_msg);

        // 2. 可选发布 PoseWithCovarianceStamped (带大协方差，帮助 EKF2 接受初始偏移)
        if (publish_cov_) {
            geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
            pose_cov_msg.header = pose_msg.header;
            pose_cov_msg.pose.pose = pose_msg.pose;
            
            std::fill(std::begin(pose_cov_msg.pose.covariance), 
                      std::end(pose_cov_msg.pose.covariance), 0.0);
            pose_cov_msg.pose.covariance[0]  = cov_xy_;   // x
            pose_cov_msg.pose.covariance[7]  = cov_xy_;   // y
            pose_cov_msg.pose.covariance[14] = cov_z_;    // z
            pose_cov_msg.pose.covariance[21] = cov_xy_;   // roll
            pose_cov_msg.pose.covariance[28] = cov_xy_;   // pitch
            pose_cov_msg.pose.covariance[35] = cov_xy_;   // yaw
            
            pose_cov_pub_->publish(pose_cov_msg);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
    rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr status_sub_;
    
    std::string frame_id_;
    bool first_msg_, fusion_confirmed_, align_origin_, publish_cov_;
    double offset_x_, offset_y_, offset_z_;
    double cov_xy_, cov_z_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryToVisionPose>());
    rclcpp::shutdown();
    return 0;
}
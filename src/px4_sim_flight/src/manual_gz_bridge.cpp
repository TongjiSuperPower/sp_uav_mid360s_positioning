#include <memory>
#include <string>
#include <random>
#include <cmath>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class ManualGzBridge : public rclcpp::Node {
public:
    ManualGzBridge() : Node("manual_gz_bridge") {
        this->declare_parameter<std::string>("gz_pose_topic", "/world/walls/pose/info");
        this->declare_parameter<std::string>("true_odom_topic", "/gz_true_odom");
        this->declare_parameter<std::string>("noisy_odom_topic", "/aft_mapped_to_init");
        this->declare_parameter<std::string>("frame_id", "camera_init");
        this->declare_parameter<std::string>("child_frame_id", "body");
        this->declare_parameter<double>("position_noise_std", 0.02);
        this->declare_parameter<double>("orientation_noise_std", 0.01);

        std::string gz_topic = this->get_parameter("gz_pose_topic").as_string();
        true_topic_ = this->get_parameter("true_odom_topic").as_string();
        noisy_topic_ = this->get_parameter("noisy_odom_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();
        pos_noise_std_ = this->get_parameter("position_noise_std").as_double();
        ori_noise_std_ = this->get_parameter("orientation_noise_std").as_double();

        pub_true_ = this->create_publisher<nav_msgs::msg::Odometry>(true_topic_, 10);
        pub_noisy_ = this->create_publisher<nav_msgs::msg::Odometry>(noisy_topic_, 10);

        gz_node_ = std::make_shared<gz::transport::Node>();

        bool ok = gz_node_->Subscribe(gz_topic, &ManualGzBridge::OnPoseVector, this);
        RCLCPP_INFO(this->get_logger(), "PoseVector sub (%s): %s", gz_topic.c_str(), ok ? "OK" : "FAIL");

        rng_.seed(std::random_device{}());
    }

private:
    void FillOdom(const gz::msgs::Pose &pose, nav_msgs::msg::Odometry &odom, bool add_noise) {
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = frame_id_;
        odom.child_frame_id = child_frame_id_;

        std::normal_distribution<double> pos_n(0.0, pos_noise_std_);
        std::normal_distribution<double> ori_n(0.0, ori_noise_std_);

        double nx = add_noise ? pos_n(rng_) : 0.0;
        double ny = add_noise ? pos_n(rng_) : 0.0;
        double nz = add_noise ? pos_n(rng_) : 0.0;

        odom.pose.pose.position.x = pose.position().x() + nx;
        odom.pose.pose.position.y = pose.position().y() + ny;
        odom.pose.pose.position.z = pose.position().z() + nz;

        double qx = pose.orientation().x() + (add_noise ? ori_n(rng_) : 0.0);
        double qy = pose.orientation().y() + (add_noise ? ori_n(rng_) : 0.0);
        double qz = pose.orientation().z() + (add_noise ? ori_n(rng_) : 0.0);
        double qw = pose.orientation().w() + (add_noise ? ori_n(rng_) : 0.0);
        double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        odom.pose.pose.orientation.x = qx / norm;
        odom.pose.pose.orientation.y = qy / norm;
        odom.pose.pose.orientation.z = qz / norm;
        odom.pose.pose.orientation.w = qw / norm;

        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
        odom.pose.covariance[0]  = pos_noise_std_ * pos_noise_std_;
        odom.pose.covariance[7]  = pos_noise_std_ * pos_noise_std_;
        odom.pose.covariance[14] = pos_noise_std_ * pos_noise_std_;
        odom.pose.covariance[21] = ori_noise_std_ * ori_noise_std_;
        odom.pose.covariance[28] = ori_noise_std_ * ori_noise_std_;
        odom.pose.covariance[35] = ori_noise_std_ * ori_noise_std_;
    }

    void OnPoseVector(const gz::msgs::Pose_V &msg) {
        for (const auto &pose : msg.pose()) {
            if (pose.name() == "x500_0") {
                nav_msgs::msg::Odometry true_odom;
                FillOdom(pose, true_odom, false);
                pub_true_->publish(true_odom);

                nav_msgs::msg::Odometry noisy_odom;
                FillOdom(pose, noisy_odom, true);
                pub_noisy_->publish(noisy_odom);
                break;
            }
        }
    }

    std::shared_ptr<gz::transport::Node> gz_node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_true_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_noisy_;
    std::mt19937 rng_;
    double pos_noise_std_, ori_noise_std_;
    std::string true_topic_, noisy_topic_, frame_id_, child_frame_id_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualGzBridge>());
    rclcpp::shutdown();
    return 0;
}

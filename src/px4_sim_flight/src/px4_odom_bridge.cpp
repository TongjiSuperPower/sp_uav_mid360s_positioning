#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <array>

class PX4OdomBridge : public rclcpp::Node {
public:
    PX4OdomBridge() : Node("px4_odom_bridge") {
        this->declare_parameter<bool>("use_gazebo_truth", false);
        this->declare_parameter<std::string>("gazebo_truth_topic", "/model/x500_0/pose");
        this->declare_parameter<std::string>("lio_topic", "/Odometry");
        
        bool use_gazebo_truth = this->get_parameter("use_gazebo_truth").as_bool();
        RCLCPP_INFO(this->get_logger(), "PARAM use_gazebo_truth = %s", use_gazebo_truth ? "TRUE" : "FALSE");
        
        pub_px4_odom_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_odometry", 10);
        RCLCPP_INFO(this->get_logger(), "PUBLISHER created: /fmu/in/vehicle_odometry");
        
        if (use_gazebo_truth) {
            std::string truth_topic = this->get_parameter("gazebo_truth_topic").as_string();
            RCLCPP_INFO(this->get_logger(), "MODE: Gazebo Truth [%s]", truth_topic.c_str());
            
            sub_truth_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                truth_topic, 10,
                std::bind(&PX4OdomBridge::truthPoseCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "SUBSCRIBER created: %s", truth_topic.c_str());
                
            last_pos_ned_ = {0.0, 0.0, 0.0};
            last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            first_msg_ = true;
        } else {
            std::string lio_topic = this->get_parameter("lio_topic").as_string();
            RCLCPP_INFO(this->get_logger(), "MODE: LIO/Odometry [%s]", lio_topic.c_str());
            
            sub_lio_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                lio_topic, 10,
                std::bind(&PX4OdomBridge::lioCallback, this, std::placeholders::_1));
        }
        
        RCLCPP_INFO(this->get_logger(), "PX4 Odom Bridge ready");
    }

private:
    void truthPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        static int cb_cnt = 0;
        if (++cb_cnt % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "[CB] truthPoseCallback called %d times", cb_cnt);
        }

        double x_ned = msg->pose.position.y;
        double y_ned = msg->pose.position.x;
        double z_ned = -msg->pose.position.z;
        
        rclcpp::Time now(msg->header.stamp);
        double dt = 0.0;
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
        
        if (!first_msg_) {
            dt = (now - last_time_).seconds();
            if (dt > 0.001 && dt < 1.0) {
                vx = static_cast<float>((x_ned - last_pos_ned_[0]) / dt);
                vy = static_cast<float>((y_ned - last_pos_ned_[1]) / dt);
                vz = static_cast<float>((z_ned - last_pos_ned_[2]) / dt);
            }
        } else {
            first_msg_ = false;
            RCLCPP_INFO(this->get_logger(), "[CB] First pose received, dt skipped");
        }
        
        last_pos_ned_ = {x_ned, y_ned, z_ned};
        last_time_ = now;
        
        tf2::Quaternion q_enu(
            msg->pose.orientation.x, msg->pose.orientation.y,
            msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Quaternion q_enu_to_ned(0.70710678, 0.70710678, 0.0, 0.0);
        tf2::Quaternion q_ned = q_enu_to_ned * q_enu;
        q_ned.normalize();
        
        px4_msgs::msg::VehicleOdometry out{};
        out.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        out.timestamp_sample = out.timestamp;
        
        out.position[0] = static_cast<float>(x_ned);
        out.position[1] = static_cast<float>(y_ned);
        out.position[2] = static_cast<float>(z_ned);
        
        out.velocity[0] = vx;
        out.velocity[1] = vy;
        out.velocity[2] = vz;
        
        out.q[0] = static_cast<float>(q_ned.w());
        out.q[1] = static_cast<float>(q_ned.x());
        out.q[2] = static_cast<float>(q_ned.y());
        out.q[3] = static_cast<float>(q_ned.z());
        
        out.angular_velocity[0] = NAN;
        out.angular_velocity[1] = NAN;
        out.angular_velocity[2] = NAN;
        
        out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        
        out.position_variance[0] = 1e-6f;
        out.position_variance[1] = 1e-6f;
        out.position_variance[2] = 1e-6f;
        out.orientation_variance[0] = 1e-6f;
        out.orientation_variance[1] = 1e-6f;
        out.orientation_variance[2] = 1e-6f;
        out.velocity_variance[0] = 0.001f;
        out.velocity_variance[1] = 0.001f;
        out.velocity_variance[2] = 0.001f;
        
        pub_px4_odom_->publish(out);
        
        if (cb_cnt % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "[PUB] Published EV odom #%d: pos=[%.2f, %.2f, %.2f]", 
                        cb_cnt, out.position[0], out.position[1], out.position[2]);
        }
    }
    
    void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        px4_msgs::msg::VehicleOdometry out{};
        out.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        out.timestamp_sample = out.timestamp;
        
        out.position[0] = static_cast<float>(msg->pose.pose.position.y);
        out.position[1] = static_cast<float>(msg->pose.pose.position.x);
        out.position[2] = static_cast<float>(-msg->pose.pose.position.z);
        
        out.velocity[0] = static_cast<float>(msg->twist.twist.linear.y);
        out.velocity[1] = static_cast<float>(msg->twist.twist.linear.x);
        out.velocity[2] = static_cast<float>(-msg->twist.twist.linear.z);
        
        tf2::Quaternion q_enu(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Quaternion q_enu_to_ned(0.70710678, 0.70710678, 0.0, 0.0);
        tf2::Quaternion q_ned = q_enu_to_ned * q_enu;
        q_ned.normalize();
        
        out.q[0] = static_cast<float>(q_ned.w());
        out.q[1] = static_cast<float>(q_ned.x());
        out.q[2] = static_cast<float>(q_ned.y());
        out.q[3] = static_cast<float>(q_ned.z());
        
        out.angular_velocity[0] = static_cast<float>(msg->twist.twist.angular.x);
        out.angular_velocity[1] = static_cast<float>(-msg->twist.twist.angular.y);
        out.angular_velocity[2] = static_cast<float>(-msg->twist.twist.angular.z);
        
        out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        
        out.position_variance[0] = 0.01f;
        out.position_variance[1] = 0.01f;
        out.position_variance[2] = 0.01f;
        out.orientation_variance[0] = 0.05f;
        out.orientation_variance[1] = 0.05f;
        out.orientation_variance[2] = 0.05f;
        out.velocity_variance[0] = 0.01f;
        out.velocity_variance[1] = 0.01f;
        out.velocity_variance[2] = 0.01f;
        
        pub_px4_odom_->publish(out);
    }
    
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_px4_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_lio_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_truth_pose_;
    
    std::array<double, 3> last_pos_ned_;
    rclcpp::Time last_time_;
    bool first_msg_ = true;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4OdomBridge>());
    rclcpp::shutdown();
    return 0;
}

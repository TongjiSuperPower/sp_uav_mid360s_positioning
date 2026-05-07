#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include <deque>
#include <cmath>
#include <Eigen/Dense>
#include <cstdio>
#include <iostream>

namespace px4_flight {

struct PositionMonitorConfig {
    double visual_ekf_xy_threshold = 0.3;
    double visual_ekf_z_threshold = 0.2;
    double hover_position_variance = 0.01;
    double hover_velocity_threshold = 0.3;
    double vibration_threshold = 0.5;
    double odom_timeout = 0.5;
};

class PositionMonitor : public rclcpp::Node {
public:
    explicit PositionMonitor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("position_monitor", options) {
        
        // 强制设置日志级别为DEBUG
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);
        
        declare_parameters();
        load_config();
        
        // BEST_EFFORT QoS (适配PX4 v1.16.0)
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        qos.history(rclcpp::HistoryPolicy::KeepLast);
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", qos,
            std::bind(&PositionMonitor::onOdometry, this, std::placeholders::_1));
            
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", qos,
            std::bind(&PositionMonitor::onImu, this, std::placeholders::_1));
            
        ekf_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&PositionMonitor::onEkfPosition, this, std::placeholders::_1));
            
        ekf_att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&PositionMonitor::onEkfAttitude, this, std::placeholders::_1));
            
        ekf_status_sub_ = create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
            "/fmu/out/estimator_status_flags", qos,
            std::bind(&PositionMonitor::onEkfStatus, this, std::placeholders::_1));
            
        vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", qos,
            std::bind(&PositionMonitor::onVehicleStatus, this, std::placeholders::_1));
        
        alert_pub_ = create_publisher<std_msgs::msg::String>("/position/monitor_alert", 10);
        drift_pub_ = create_publisher<std_msgs::msg::Float64>("/position/visual_ekf_drift", 10);
        stability_pub_ = create_publisher<std_msgs::msg::Float64>("/position/hover_stability", 10);
        
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PositionMonitor::publishReport, this));
            
        printf("[MONITOR] Position Monitor initialized - ALL LOGS PRINT TO TERMINAL\n");
        fflush(stdout);
        RCLCPP_INFO(this->get_logger(), "Position Monitor initialized");
    }

private:
    void declare_parameters() {
        this->declare_parameter("visual_ekf_xy_threshold", 0.3);
        this->declare_parameter("visual_ekf_z_threshold", 0.2);
        this->declare_parameter("hover_position_variance", 0.01);
        this->declare_parameter("hover_velocity_threshold", 0.3);
        this->declare_parameter("vibration_threshold", 0.5);
        this->declare_parameter("odom_timeout", 0.5);
    }
    
    void load_config() {
        config_.visual_ekf_xy_threshold = this->get_parameter("visual_ekf_xy_threshold").as_double();
        config_.visual_ekf_z_threshold = this->get_parameter("visual_ekf_z_threshold").as_double();
        config_.hover_position_variance = this->get_parameter("hover_position_variance").as_double();
        config_.hover_velocity_threshold = this->get_parameter("hover_velocity_threshold").as_double();
        config_.vibration_threshold = this->get_parameter("vibration_threshold").as_double();
        config_.odom_timeout = this->get_parameter("odom_timeout").as_double();
    }

    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_odom_time_ = this->now();
        visual_position_ = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        Eigen::Vector3d vel(msg->twist.twist.linear.x,
                           msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
        odom_window_.push_back({last_odom_time_, visual_position_, vel});
        if (odom_window_.size() > 100) odom_window_.pop_front();
    }
    
    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double acc_norm = std::sqrt(
            msg->linear_acceleration.x * msg->linear_acceleration.x +
            msg->linear_acceleration.y * msg->linear_acceleration.y +
            msg->linear_acceleration.z * msg->linear_acceleration.z);
        double acc_body = std::abs(acc_norm - 9.81);
        imu_acc_window_.push_back(acc_body);
        if (imu_acc_window_.size() > 200) imu_acc_window_.pop_front();
    }
    
    void onEkfPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_ekf_time_ = this->now();
        ekf_position_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
        ekf_velocity_ = Eigen::Vector3d(msg->vx, msg->vy, msg->vz);
        ekf_xy_valid_ = msg->xy_valid;
        ekf_z_valid_ = msg->z_valid;
    }
    
    void onEkfAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_attitude_time_ = this->now();
        ekf_attitude_ = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    }
    
    void onEkfStatus(const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ekf_ev_pos_ = msg->cs_ev_pos;
        ekf_ev_yaw_ = msg->cs_ev_yaw;
        ekf_ev_hgt_ = msg->cs_ev_hgt;
        ekf_ev_vel_ = msg->cs_ev_vel;
        reject_yaw_ = msg->reject_yaw;
        reject_hor_pos_ = msg->reject_hor_pos;
        reject_ver_pos_ = msg->reject_ver_pos;
    }
    
    void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        is_position_mode_ = (msg->nav_state == 2);
        is_armed_ = (msg->arming_state == 2);
    }

    void checkVisualEkfDrift() {
        if (!ekf_xy_valid_ || !ekf_z_valid_) return;
        Eigen::Vector3d diff = visual_position_ - ekf_position_;
        double xy_drift = std::sqrt(diff.x() * diff.x() + diff.y() * diff.y());
        double z_drift = std::abs(diff.z());
        
        std_msgs::msg::Float64 drift_msg;
        drift_msg.data = xy_drift;
        drift_pub_->publish(drift_msg);
        
        if (xy_drift > config_.visual_ekf_xy_threshold) {
            logAlert(WARN, "Visual-EKF XY drift: %.2fm (threshold: %.2fm)", xy_drift, config_.visual_ekf_xy_threshold);
        }
        if (z_drift > config_.visual_ekf_z_threshold) {
            logAlert(WARN, "Visual-EKF Z drift: %.2fm (threshold: %.2fm)", z_drift, config_.visual_ekf_z_threshold);
        }
    }
    
    void checkHoverStability() {
        if (odom_window_.size() < 30) return;
        Eigen::Vector3d mean(0, 0, 0);
        for (const auto& s : odom_window_) mean += s.position;
        mean /= odom_window_.size();
        
        double var = 0;
        for (const auto& s : odom_window_) {
            Eigen::Vector3d d = s.position - mean;
            var += d.squaredNorm();
        }
        var /= odom_window_.size();
        
        std_msgs::msg::Float64 stab_msg;
        stab_msg.data = var;
        stability_pub_->publish(stab_msg);
        
        if (var > config_.hover_position_variance) {
            logAlert(INFO, "Hover stability: var=%.4f (drifting)", var);
        }
        
        double mean_speed = 0;
        for (const auto& s : odom_window_) mean_speed += s.velocity.norm();
        mean_speed /= odom_window_.size();
        
        if (mean_speed > config_.hover_velocity_threshold) {
            logAlert(INFO, "Hover speed: %.2fm/s (moving)", mean_speed);
        }
    }
    
    void checkEkfHealth() {
        if (!ekf_ev_pos_ || !ekf_ev_hgt_) {
            logAlert(ERROR, "EKF2 visual fusion lost! pos=%d hgt=%d", ekf_ev_pos_, ekf_ev_hgt_);
        }
        if (reject_yaw_) {
            logAlert(WARN, "EKF2 rejecting yaw");
        }
        if (reject_hor_pos_) {
            logAlert(ERROR, "EKF2 rejecting horizontal position!");
        }
        if (reject_ver_pos_) {
            logAlert(WARN, "EKF2 rejecting vertical position");
        }
    }
    
    void checkVibration() {
        if (imu_acc_window_.size() < 100) return;
        double mean = 0;
        for (double v : imu_acc_window_) mean += v;
        mean /= imu_acc_window_.size();
        
        double var = 0;
        for (double v : imu_acc_window_) var += (v - mean) * (v - mean);
        var /= imu_acc_window_.size();
        
        if (var > config_.vibration_threshold) {
            logAlert(WARN, "High vibration: var=%.3f", var);
        }
    }
    
    void checkDataFreshness() {
        auto now = this->now();
        double odom_age = (now - last_odom_time_).seconds();
        if (odom_age > config_.odom_timeout) {
            logAlert(ERROR, "Odometry timeout: %.1fs ago!", odom_age);
        }
        double ekf_age = (now - last_ekf_time_).seconds();
        if (ekf_age > config_.odom_timeout) {
            logAlert(ERROR, "EKF position timeout: %.1fs ago!", ekf_age);
        }
    }

    void publishReport() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!is_position_mode_ || !is_armed_) {
            printf("[MONITOR] Waiting for Position mode + ARMED (mode=%d, armed=%d)\n", 
                   is_position_mode_, is_armed_);
            fflush(stdout);
            return;
        }
        checkDataFreshness();
        checkVisualEkfDrift();
        checkHoverStability();
        checkEkfHealth();
        checkVibration();
    }

    enum AlertLevel { INFO=0, WARN=1, ERROR=2 };
    
    void logAlert(AlertLevel level, const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        
        const char* level_str = levelToString(level);
        
        // 【关键】强制终端输出
        printf("[%s] %s\n", level_str, buffer);
        fflush(stdout);
        
        // ROS2日志
        std_msgs::msg::String msg;
        msg.data = std::string("[") + level_str + "] " + buffer;
        alert_pub_->publish(msg);
        
        switch (level) {
            case INFO: RCLCPP_INFO(this->get_logger(), "%s", buffer); break;
            case WARN: RCLCPP_WARN(this->get_logger(), "%s", buffer); break;
            case ERROR: RCLCPP_ERROR(this->get_logger(), "%s", buffer); break;
        }
        va_end(args);
    }
    
    const char* levelToString(AlertLevel level) {
        switch (level) {
            case INFO: return "INFO";
            case WARN: return "WARN";
            case ERROR: return "ERROR";
        }
        return "UNKNOWN";
    }

    struct OdomSample {
        rclcpp::Time stamp;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
    };
    
    PositionMonitorConfig config_;
    std::mutex data_mutex_;
    std::deque<OdomSample> odom_window_;
    std::deque<double> imu_acc_window_;
    
    Eigen::Vector3d visual_position_{0, 0, 0};
    Eigen::Vector3d ekf_position_{0, 0, 0};
    Eigen::Vector3d ekf_velocity_{0, 0, 0};
    Eigen::Quaterniond ekf_attitude_{1, 0, 0, 0};
    
    bool ekf_xy_valid_ = false;
    bool ekf_z_valid_ = false;
    bool ekf_ev_pos_ = false;
    bool ekf_ev_yaw_ = false;
    bool ekf_ev_hgt_ = false;
    bool ekf_ev_vel_ = false;
    bool reject_yaw_ = false;
    bool reject_hor_pos_ = false;
    bool reject_ver_pos_ = false;
    bool is_position_mode_ = false;
    bool is_armed_ = false;
    
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_ekf_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_attitude_time_{0, 0, RCL_ROS_TIME};
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drift_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stability_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr ekf_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr ekf_att_sub_;
    rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr ekf_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace px4_flight

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_flight::PositionMonitor>());
    rclcpp::shutdown();
    return 0;
}

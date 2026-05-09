#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

using namespace std::chrono_literals;

class OffboardLandingNode : public rclcpp::Node {
public:
    OffboardLandingNode() : Node("offboard_landing_node") {
        this->declare_parameter<double>("takeoff_height", 2.0);
        this->declare_parameter<double>("hover_time", 3.0);
        this->declare_parameter<double>("approach_velocity", 0.5);
        this->declare_parameter<double>("descent_velocity", -0.3);
        this->declare_parameter<double>("final_descent_height", 0.5);
        this->declare_parameter<double>("stop_adjust_height", 0.3);
        this->declare_parameter<double>("final_descent_velocity", -0.2);
        this->declare_parameter<std::string>("qr_topic", "/qr_land/pose");
        this->declare_parameter<double>("qr_position_x", 0.0);
        this->declare_parameter<double>("qr_position_y", 2.0);
        
        takeoff_height_ = this->get_parameter("takeoff_height").as_double();
        hover_time_ = this->get_parameter("hover_time").as_double();
        approach_velocity_ = this->get_parameter("approach_velocity").as_double();
        descent_velocity_ = this->get_parameter("descent_velocity").as_double();
        final_descent_height_ = this->get_parameter("final_descent_height").as_double();
        stop_adjust_height_ = this->get_parameter("stop_adjust_height").as_double();
        final_descent_velocity_ = this->get_parameter("final_descent_velocity").as_double();
        qr_topic_ = this->get_parameter("qr_topic").as_string();
        qr_x_ = this->get_parameter("qr_position_x").as_double();
        qr_y_ = this->get_parameter("qr_position_y").as_double();

        pub_offboard_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        pub_trajectory_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        pub_vehicle_cmd_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        sub_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 10,
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                current_status_ = *msg;
                status_received_ = true;
            });
            
        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 10,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                current_pos_ = *msg;
                pos_received_ = true;
            });
            
        sub_qr_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            qr_topic_, 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                qr_detected_x_ = msg->pose.position.y;
                qr_detected_y_ = msg->pose.position.x;
                qr_detected_z_ = -msg->pose.position.z;
                qr_detected_ = true;
                last_qr_time_ = this->now();
            });

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardLandingNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Offboard Landing Node initialized");
    }

private:
    enum class State { INIT, ARMING, TAKEOFF, HOVER, QR_TRACK, DESCENT, FINAL_DESCENT, LANDED };
    
    void timerCallback() {
        if (!status_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for PX4 vehicle_status...");
            return;
        }

        if (qr_detected_ && (this->now() - last_qr_time_).seconds() > 1.0) {
            qr_detected_ = false;
            RCLCPP_WARN(this->get_logger(), "QR code lost, using last known position");
        }

        publishOffboardControlMode();
        
        switch (state_) {
            case State::INIT:
                if (status_received_) {
                    RCLCPP_INFO(this->get_logger(), "PX4 connected, start arming sequence...");
                    state_ = State::ARMING;
                    arm_attempts_ = 0;
                }
                break;
                
            case State::ARMING:
                if (current_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                    RCLCPP_INFO(this->get_logger(), "Armed! Takeoff...");
                    state_ = State::TAKEOFF;
                } else {
                    if (arm_attempts_++ % 10 == 0) {
                        publishVehicleCommand(
                            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            1.0, 21196.0);
                        RCLCPP_INFO(this->get_logger(), "Sending ARM command...");
                    }
                    publishTrajectorySetpoint(0, 0, -0.1, 0);
                }
                break;
                
            case State::TAKEOFF:
                publishTrajectorySetpoint(0, 0, -takeoff_height_, 0);
                if (std::abs(current_pos_.z + takeoff_height_) < 0.2) {
                    RCLCPP_INFO(this->get_logger(), "Takeoff complete, hovering...");
                    state_ = State::HOVER;
                    hover_start_time_ = this->now();
                }
                break;
                
            case State::HOVER:
                publishTrajectorySetpoint(0, 0, -takeoff_height_, 0);
                if ((this->now() - hover_start_time_).seconds() > hover_time_) {
                    RCLCPP_INFO(this->get_logger(), "Hover complete, entering OFFBOARD QR tracking...");
                    publishVehicleCommand(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                        1.0, 6.0);
                    state_ = State::QR_TRACK;
                }
                break;
                
            case State::QR_TRACK: {
                double target_x = qr_detected_ ? qr_detected_x_ : qr_x_;
                double target_y = qr_detected_ ? qr_detected_y_ : qr_y_;
                
                double dx = target_x - current_pos_.x;
                double dy = target_y - current_pos_.y;
                double dist = std::sqrt(dx*dx + dy*dy);
                
                publishTrajectorySetpoint(target_x, target_y, -takeoff_height_, 0);
                
                if (dist < 0.3 && qr_detected_) {
                    RCLCPP_INFO(this->get_logger(), "QR aligned (err=%.2fm), start descent...", dist);
                    state_ = State::DESCENT;
                }
                break;
            }
            
            case State::DESCENT: {
                double target_x = qr_detected_ ? qr_detected_x_ : qr_x_;
                double target_y = qr_detected_ ? qr_detected_y_ : qr_y_;
                double target_z = -(qr_detected_ ? (qr_detected_z_ + final_descent_height_) : final_descent_height_);
                
                publishTrajectorySetpoint(target_x, target_y, target_z, 0);
                
                if (std::abs(current_pos_.z - target_z) < 0.1) {
                    RCLCPP_INFO(this->get_logger(), "Reached %.1fm, entering final descent...", final_descent_height_);
                    state_ = State::FINAL_DESCENT;
                    final_x_ = current_pos_.x;
                    final_y_ = current_pos_.y;
                }
                break;
            }
            
            case State::FINAL_DESCENT: {
                double current_height = -current_pos_.z;
                double rel_height = qr_detected_ ? qr_detected_z_ : current_height;
                
                if (rel_height < stop_adjust_height_) {
                    publishTrajectoryVelocity(final_x_, final_y_, final_descent_velocity_, 0);
                } else {
                    double target_x = qr_detected_ ? qr_detected_x_ : final_x_;
                    double target_y = qr_detected_ ? qr_detected_y_ : final_y_;
                    double target_z = -(qr_detected_ ? (qr_detected_z_ + 0.05) : 0.05);
                    publishTrajectorySetpoint(target_x, target_y, target_z, 0);
                }
                
                if (current_height < 0.15) {
                    RCLCPP_INFO(this->get_logger(), "Touchdown detected. Disarming...");
                    state_ = State::LANDED;
                }
                break;
            }
            
            case State::LANDED:
                if (land_attempts_++ % 10 == 0) {
                    publishVehicleCommand(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        0.0, 0.0);
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Landed and disarmed.");
                break;
        }
    }
    
    void publishOffboardControlMode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        pub_offboard_mode_->publish(msg);
    }
    
    void publishTrajectorySetpoint(double x, double y, double z, double yaw) {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position[0] = static_cast<float>(x);
        msg.position[1] = static_cast<float>(y);
        msg.position[2] = static_cast<float>(z);
        msg.yaw = static_cast<float>(yaw);
        pub_trajectory_->publish(msg);
    }
    
    void publishTrajectoryVelocity(double x, double y, double vz, double yaw) {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position[0] = static_cast<float>(x);
        msg.position[1] = static_cast<float>(y);
        msg.position[2] = NAN;
        msg.velocity[0] = 0;
        msg.velocity[1] = 0;
        msg.velocity[2] = static_cast<float>(vz);
        msg.yaw = static_cast<float>(yaw);
        pub_trajectory_->publish(msg);
    }
    
    void publishVehicleCommand(uint16_t command, double param1, double param2, double param3 = 0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.command = command;
        msg.param1 = static_cast<float>(param1);
        msg.param2 = static_cast<float>(param2);
        msg.param3 = static_cast<float>(param3);
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        pub_vehicle_cmd_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_cmd_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_qr_pose_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    State state_ = State::INIT;
    px4_msgs::msg::VehicleStatus current_status_;
    px4_msgs::msg::VehicleLocalPosition current_pos_;
    bool status_received_ = false;
    bool pos_received_ = false;
    bool qr_detected_ = false;
    double qr_detected_x_ = 0, qr_detected_y_ = 0, qr_detected_z_ = 0;
    rclcpp::Time last_qr_time_;
    
    double takeoff_height_, hover_time_, approach_velocity_, descent_velocity_;
    double final_descent_height_, stop_adjust_height_, final_descent_velocity_;
    std::string qr_topic_;
    double qr_x_, qr_y_;
    
    int arm_attempts_ = 0;
    int land_attempts_ = 0;
    rclcpp::Time hover_start_time_;
    double final_x_ = 0, final_y_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardLandingNode>());
    rclcpp::shutdown();
    return 0;
}

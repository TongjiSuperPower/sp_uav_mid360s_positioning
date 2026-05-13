#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <cmath>

class OffboardLandingTest : public rclcpp::Node
{
public:
    OffboardLandingTest() : Node("offboard_landing_test"),
        current_mode_(""), prev_mode_(""),
        init_x_(0.0), init_y_(0.0), current_z_(1.0), target_z_(1.0),
        landing_active_(false), touchdown_detected_(false),
        state_recv_(false), pos_recv_(false), ext_state_recv_(false),
        prev_landed_state_(mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED),
        last_z_(0.0), z_stagnation_time_(0.0)
    {
        // --- 参数声明 ---
        this->declare_parameter<double>("descent_rate_fast", 0.30);   // >0.5m 时的下降速度
        this->declare_parameter<double>("descent_rate_slow", 0.10);   // 0.3~0.5m 时的下降速度
        this->declare_parameter<double>("descent_rate_final", 0.05);  // <0.3m 时的下降速度
        this->declare_parameter<double>("slow_zone_height", 0.50);    // 开始减速的高度
        this->declare_parameter<double>("touchdown_height", 0.30);    // 开始极慢速的高度
        this->declare_parameter<double>("min_height", 0.05);            // 物理最低保护高度
        this->declare_parameter<double>("publish_rate", 10.0);          // 设定点发布频率

        fast_rate_   = this->get_parameter("descent_rate_fast").as_double();
        slow_rate_   = this->get_parameter("descent_rate_slow").as_double();
        final_rate_  = this->get_parameter("descent_rate_final").as_double();
        slow_zone_h_ = this->get_parameter("slow_zone_height").as_double();
        touchdown_h_ = this->get_parameter("touchdown_height").as_double();
        min_height_  = this->get_parameter("min_height").as_double();
        double pub_rate = this->get_parameter("publish_rate").as_double();
        dt_ = 1.0 / pub_rate;

        // --- QoS：使用 best_effort 匹配 MAVROS2 ---
        auto qos_be = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // --- 订阅 ---
        sub_state_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_be,
            std::bind(&OffboardLandingTest::onState, this, std::placeholders::_1));

        sub_local_pos_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_be,
            std::bind(&OffboardLandingTest::onLocalPos, this, std::placeholders::_1));

        sub_ext_state_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
            "/mavros/extended_state", qos_be,
            std::bind(&OffboardLandingTest::onExtendedState, this, std::placeholders::_1));

        // --- 发布 ---
        pub_setpoint_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 10);

        // --- 服务：上锁/解锁 ---
        cli_arming_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        // --- 定时器 ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / pub_rate)),
            std::bind(&OffboardLandingTest::timerCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "Landing controller ready: fast=%.2f slow=%.2f final=%.2f m/s | slow_zone=%.2f touch=%.2f m",
            fast_rate_, slow_rate_, final_rate_, slow_zone_h_, touchdown_h_);
        RCLCPP_INFO(this->get_logger(),
            "流程: Position悬停 → 拨杆Offboard(自动降落) → 触地自动上锁 | 拨杆回Position随时停止");
    }

private:
    // --- 回调 ---

    void onState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_mode_ = msg->mode;
        state_recv_ = true;
    }

    void onLocalPos(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_z_ = msg->pose.position.z;
        if (!pos_recv_) {
            init_x_ = msg->pose.position.x;
            init_y_ = msg->pose.position.y;
            target_z_ = current_z_;
            last_z_ = current_z_;
            pos_recv_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Position locked: X=%.3f Y=%.3f Z=%.3f", init_x_, init_y_, target_z_);
        }
    }

    void onExtendedState(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
    {
        uint8_t ls = msg->landed_state;
        ext_state_recv_ = true;

        // 触地检测：从空中/起飞/降落状态 变为 地面状态
        if (landing_active_ && !touchdown_detected_) {
            if (ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
                if (prev_landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR ||
                    prev_landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_TAKEOFF ||
                    prev_landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING) {
                    
                    touchdown_detected_ = true;
                    landing_active_ = false;
                    RCLCPP_WARN(this->get_logger(),
                        ">>> TOUCHDOWN DETECTED (ExtendedState). Sending DISARM...");
                    sendDisarm();
                }
            }
        }
        prev_landed_state_ = ls;
    }

    // --- 辅助函数 ---

    double getDescentRate(double z) const
    {
        if (z > slow_zone_h_)       return fast_rate_;
        else if (z > touchdown_h_)  return slow_rate_;
        else                        return final_rate_;
    }

    void sendDisarm()
    {
        if (!cli_arming_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Arming service not available");
            return;
        }
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = false;  // disarm
        cli_arming_->async_send_request(req,
            [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                try {
                    auto resp = future.get();
                    if (resp->success) {
                        RCLCPP_WARN(this->get_logger(), ">>> DISARM SUCCESS");
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Disarm rejected by FCU");
                    }
                } catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "Disarm service call failed");
                }
            });
    }

    void publishSetpoint()
    {
        mavros_msgs::msg::PositionTarget sp;
        sp.header.stamp = this->now();
        sp.header.frame_id = "map";
        sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        sp.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        sp.position.x = init_x_;      // 锁定，禁止横向移动
        sp.position.y = init_y_;      // 锁定，禁止横向移动
        sp.position.z = target_z_;
        sp.yaw = 0.0;

        pub_setpoint_->publish(sp);
    }

    void checkStagnationBackup()
    {
        // 备用检测：高度 < 0.15m 且连续 2 秒变化 < 0.005m，判定触地
        if (current_z_ < 0.15 && landing_active_ && !touchdown_detected_) {
            if (std::abs(current_z_ - last_z_) < 0.005) {
                z_stagnation_time_ += dt_;
                if (z_stagnation_time_ > 2.0) {
                    touchdown_detected_ = true;
                    landing_active_ = false;
                    RCLCPP_WARN(this->get_logger(),
                        ">>> TOUCHDOWN DETECTED (height stagnation backup). Sending DISARM...");
                    sendDisarm();
                }
            } else {
                z_stagnation_time_ = 0.0;
            }
        } else {
            z_stagnation_time_ = 0.0;
        }
        last_z_ = current_z_;
    }

    // --- 主定时器 ---

    void timerCallback()
    {
        if (!pos_recv_ || !state_recv_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for topics... state=%s pos=%s",
                state_recv_ ? "OK" : "NO", pos_recv_ ? "OK" : "NO");
            return;
        }

        // 模式切换检测
        if (current_mode_ != prev_mode_) {
            if (current_mode_ == "OFFBOARD") {
                RCLCPP_WARN(this->get_logger(), ">>> OFFBOARD detected, START landing");
                landing_active_ = true;
                touchdown_detected_ = false;
                z_stagnation_time_ = 0.0;
            } else if (prev_mode_ == "OFFBOARD") {
                RCLCPP_WARN(this->get_logger(),
                    ">>> OFFBOARD lost, switch to %s, HOLD position", current_mode_.c_str());
                landing_active_ = false;
                touchdown_detected_ = false;
                target_z_ = current_z_;   // 冻结当前高度
                z_stagnation_time_ = 0.0;
            }
            prev_mode_ = current_mode_;
        }

        // 已触地，不再计算，仅保持当前位置（或停止发布由飞控接管）
        if (touchdown_detected_) {
            // 继续发布当前位置作为保持，直到上锁完成
            target_z_ = current_z_;
            publishSetpoint();
            return;
        }

        // 降落逻辑
        if (landing_active_) {
            double rate = getDescentRate(current_z_);
            target_z_ -= rate * dt_;

            // 物理最低保护
            if (target_z_ < min_height_) {
                target_z_ = min_height_;
            }

            // 备用触地检测
            checkStagnationBackup();

            // 日志：阶段切换提示
            static bool logged_slow = false;
            static bool logged_final = false;
            if (!logged_slow && current_z_ <= slow_zone_h_ && current_z_ > touchdown_h_) {
                RCLCPP_INFO(this->get_logger(), "Entering SLOW zone (%.2f m/s)", slow_rate_);
                logged_slow = true;
            }
            if (!logged_final && current_z_ <= touchdown_h_) {
                RCLCPP_INFO(this->get_logger(), "Entering FINAL zone (%.2f m/s)", final_rate_);
                logged_final = true;
            }
            if (current_z_ > slow_zone_h_) { logged_slow = false; logged_final = false; }
        }

        publishSetpoint();
    }

    // --- 成员变量 ---
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr sub_ext_state_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_setpoint_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr cli_arming_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string current_mode_, prev_mode_;
    double init_x_, init_y_, current_z_, target_z_;
    double fast_rate_, slow_rate_, final_rate_;
    double slow_zone_h_, touchdown_h_, min_height_;
    double dt_, last_z_, z_stagnation_time_;
    bool landing_active_, touchdown_detected_;
    bool state_recv_, pos_recv_, ext_state_recv_;
    uint8_t prev_landed_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardLandingTest>());
    rclcpp::shutdown();
    return 0;
}
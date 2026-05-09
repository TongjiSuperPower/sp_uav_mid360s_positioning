#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <iomanip>
#include <sstream>

class FlightMonitor : public rclcpp::Node
{
public:
    FlightMonitor() : Node("flight_monitor")
    {
        // 订阅关键话题
        sub_estimator_ = this->create_subscription<mavros_msgs::msg::EstimatorStatus>(
            "/mavros/estimator_status", 10,
            std::bind(&FlightMonitor::onEstimatorStatus, this, std::placeholders::_1));

        sub_state_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&FlightMonitor::onState, this, std::placeholders::_1));

        sub_local_pos_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", 10,
            std::bind(&FlightMonitor::onLocalPosition, this, std::placeholders::_1));

        sub_battery_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", 10,
            std::bind(&FlightMonitor::onBattery, this, std::placeholders::_1));

        sub_ext_state_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
            "/mavros/extended_state", 10,
            std::bind(&FlightMonitor::onExtendedState, this, std::placeholders::_1));

        sub_vision_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10,
            std::bind(&FlightMonitor::onVisionPose, this, std::placeholders::_1));

        // 2秒定时打印
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&FlightMonitor::printStatus, this));

        RCLCPP_INFO(this->get_logger(), "Flight monitor started, printing every 2s");
    }

private:
    void onEstimatorStatus(const mavros_msgs::msg::EstimatorStatus::SharedPtr msg)
    {
        est_ = *msg;
        est_recv_ = true;
    }

    void onState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        state_ = *msg;
        state_recv_ = true;
    }

    void onLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        local_pos_ = *msg;
        local_pos_recv_ = true;
    }

    void onBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        battery_ = *msg;
        battery_recv_ = true;
    }

    void onExtendedState(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
    {
        ext_state_ = *msg;
        ext_state_recv_ = true;
    }

    void onVisionPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        vision_pose_ = *msg;
        vision_recv_ = true;
        vision_stamp_ = this->now();
    }

    std::string landedStateString(uint8_t s)
    {
        switch (s) {
            case mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED: return "未知";
            case mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND: return "地面";
            case mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR: return "空中";
            case mavros_msgs::msg::ExtendedState::LANDED_STATE_TAKEOFF: return "起飞中";
            case mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING: return "降落中";
            default: return "未知";
        }
    }

    std::string flagString(bool v) { return v ? "\033[32mOK\033[0m" : "\033[31mNO\033[0m"; }

    void printStatus()
    {
        auto now = this->now();
        std::ostringstream oss;
        oss << "\n================== 飞行状态监视 (" << std::fixed << std::setprecision(1)
            << now.seconds() << ") ==================\n";

        // 1. MAVROS 连接与飞行模式
        if (state_recv_) {
            oss << "[连接状态] "
                << (state_.connected ? "已连接" : "\033[31m未连接\033[0m") << " | "
                << (state_.armed ? "\033[33m已解锁\033[0m" : "已锁定") << " | "
                << "模式: " << state_.mode << " | "
                << "系统状态: " << (int)state_.system_status << "\n";
        } else {
            oss << "[连接状态] \033[31m未收到 /mavros/state\033[0m\n";
        }

        // 2. EKF2 融合状态
        if (est_recv_) {
            oss << "[EKF2融合] 姿态:" << flagString(est_.attitude_status_flag)
                << " 水平位置:" << flagString(est_.pos_horiz_rel_status_flag)
                << " 绝对水平:" << flagString(est_.pos_horiz_abs_status_flag)
                << " 垂直位置:" << flagString(est_.pos_vert_abs_status_flag)
                << " 恒定位置模式:" << (est_.const_pos_mode_status_flag ? "是" : "否")
                << "\n";
        } else {
            oss << "[EKF2融合] \033[31m未收到 /mavros/estimator_status\033[0m\n";
        }

        // 3. 本地位置 (EKF2 融合输出)
        if (local_pos_recv_) {
            oss << "[本地位置] X:" << std::setprecision(3) << local_pos_.pose.position.x
                << " Y:" << local_pos_.pose.position.y
                << " Z:" << local_pos_.pose.position.z << "\n";
        } else {
            oss << "[本地位置] \033[31m未收到\033[0m\n";
        }

        // 4. 外部视觉输入 (FAST-LIO2 -> MAVROS2)
        bool vision_alive = vision_recv_ && (now - vision_stamp_).seconds() < 1.0;
        if (vision_alive) {
            oss << "[外部视觉] 数据流: \033[32m正常\033[0m | "
                << "输入X:" << std::setprecision(3) << vision_pose_.pose.position.x
                << " Y:" << vision_pose_.pose.position.y
                << " Z:" << vision_pose_.pose.position.z << "\n";
        } else if (vision_recv_) {
            oss << "[外部视觉] 数据流: \033[31m中断 (>1s)\033[0m\n";
        } else {
            oss << "[外部视觉] 数据流: \033[31m未收到\033[0m\n";
        }

        // 5. 电池
        if (battery_recv_) {
            oss << "[电池状态] 电压:" << std::setprecision(2) << battery_.voltage << "V"
                << " | 电流:" << std::setprecision(2) << battery_.current << "A"
                << " | 剩余:" << std::setprecision(1) << (battery_.percentage * 100.0) << "%"
                << (battery_.voltage < 14.8 ? " \033[31m[低压警告]\033[0m" : "")
                << "\n";
        } else {
            oss << "[电池状态] 未收到\n";
        }

        // 6. 起落架/飞行阶段
        if (ext_state_recv_) {
            oss << "[飞行阶段] " << landedStateString(ext_state_.landed_state) << "\n";
        } else {
            oss << "[飞行阶段] 未收到\n";
        }

        oss << "============================================================\n";
        std::cout << oss.str() << std::flush;
    }

    // 订阅者
    rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr sub_estimator_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr sub_ext_state_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_vision_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 数据缓存
    mavros_msgs::msg::EstimatorStatus est_;
    mavros_msgs::msg::State state_;
    geometry_msgs::msg::PoseStamped local_pos_;
    sensor_msgs::msg::BatteryState battery_;
    mavros_msgs::msg::ExtendedState ext_state_;
    geometry_msgs::msg::PoseStamped vision_pose_;
    rclcpp::Time vision_stamp_;

    bool est_recv_ = false;
    bool state_recv_ = false;
    bool local_pos_recv_ = false;
    bool battery_recv_ = false;
    bool ext_state_recv_ = false;
    bool vision_recv_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightMonitor>());
    rclcpp::shutdown();
    return 0;
}
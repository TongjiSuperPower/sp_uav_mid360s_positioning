#include <memory>
#include <string>
#include <cstring>
#include <array>

#include <gz/transport/Node.hh>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/camera_info.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class ManualGzBridge : public rclcpp::Node {
public:
    ManualGzBridge() : Node("manual_gz_bridge") {
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/x500_depth_0/pose", 10);
        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/x500_depth_0/camera/image", 10);
        pub_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/x500_depth_0/camera/camera_info", 10);
        pub_clock_ = this->create_publisher<rosgraph_msgs::msg::Clock>(
            "/clock", 10);

        gz_node_ = std::make_shared<gz::transport::Node>();

        bool ok1 = gz_node_->Subscribe("/world/default/pose/info", &ManualGzBridge::OnPoseVector, this);
        bool ok2 = gz_node_->Subscribe(
            "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
            &ManualGzBridge::OnImage, this);
        bool ok3 = gz_node_->Subscribe("/camera_info", &ManualGzBridge::OnCameraInfo, this);
        bool ok4 = gz_node_->Subscribe("/clock", &ManualGzBridge::OnClock, this);

        RCLCPP_INFO(this->get_logger(), "PoseVector sub: %s", ok1 ? "OK" : "FAIL");
        RCLCPP_INFO(this->get_logger(), "RGB Image sub:  %s", ok2 ? "OK" : "FAIL");
        RCLCPP_INFO(this->get_logger(), "CameraInfo sub: %s", ok3 ? "OK" : "FAIL");
        RCLCPP_INFO(this->get_logger(), "Clock sub:      %s", ok4 ? "OK" : "FAIL");
    }

private:
    void OnPoseVector(const gz::msgs::Pose_V &msg) {
        static int cnt = 0;
        for (const auto &pose : msg.pose()) {
            if (pose.name() == "x500_depth_0") {
                if (++cnt % 100 == 0) RCLCPP_INFO(this->get_logger(), "Pose recv: %d", cnt);
                geometry_msgs::msg::PoseStamped out;
                out.header.stamp = this->get_clock()->now();
                out.header.frame_id = "map";
                out.pose.position.x = pose.position().x();
                out.pose.position.y = pose.position().y();
                out.pose.position.z = pose.position().z();
                out.pose.orientation.x = pose.orientation().x();
                out.pose.orientation.y = pose.orientation().y();
                out.pose.orientation.z = pose.orientation().z();
                out.pose.orientation.w = pose.orientation().w();
                pub_pose_->publish(out);
                break;
            }
        }
    }

    void OnImage(const gz::msgs::Image &msg) {
        static int cnt = 0;
        if (++cnt % 30 == 0) RCLCPP_INFO(this->get_logger(), "Image recv: %d", cnt);

        sensor_msgs::msg::Image out;
        out.header.stamp = this->get_clock()->now();
        out.header.frame_id = "camera_link";
        out.width = msg.width();
        out.height = msg.height();
        out.step = msg.step();
        out.encoding = "rgb8";
        out.is_bigendian = 0;
        out.data.resize(msg.data().size());
        memcpy(out.data.data(), msg.data().data(), msg.data().size());
        pub_image_->publish(out);

        publishMatchedCameraInfo(msg.width(), msg.height());
    }

    void publishMatchedCameraInfo(uint32_t w, uint32_t h) {
        double sx = static_cast<double>(w) / 640.0;
        double sy = static_cast<double>(h) / 480.0;

        sensor_msgs::msg::CameraInfo out;
        out.header.stamp = this->get_clock()->now();
        out.header.frame_id = "camera_link";
        out.width = w;
        out.height = h;
        out.distortion_model = "plumb_bob";
        out.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        // 显式初始化 std::array<double, 9> k
        out.k = std::array<double, 9>{
            432.496042035043 * sx, 0.0,                    320.0 * sx,
            0.0,                   432.496042035043 * sy, 240.0 * sy,
            0.0,                   0.0,                    1.0
        };

        // 显式初始化 std::array<double, 12> p
        out.p = std::array<double, 12>{
            out.k[0], 0.0, out.k[2], 0.0,
            0.0,      out.k[4], out.k[5], 0.0,
            0.0,      0.0,      1.0,      0.0
        };

        // 显式初始化 std::array<double, 9> r (单位矩阵)
        out.r = std::array<double, 9>{
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        pub_camera_info_->publish(out);
    }

    void OnCameraInfo(const gz::msgs::CameraInfo &msg) { (void)msg; }

    void OnClock(const gz::msgs::Clock &msg) {
        rosgraph_msgs::msg::Clock out;
        out.clock.sec = msg.sim().sec();
        out.clock.nanosec = msg.sim().nsec();
        pub_clock_->publish(out);
    }

    std::shared_ptr<gz::transport::Node> gz_node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualGzBridge>());
    rclcpp::shutdown();
    return 0;
}

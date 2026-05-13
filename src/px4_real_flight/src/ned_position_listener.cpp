#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class NEDPositionListener : public rclcpp::Node
{
public:
  NEDPositionListener() : Node("ned_position_listener")
  {
    // 订阅 Fast-LIO2 的 /Odometry（ENU 坐标系）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          // 直接读取 ENU 的 x/y/z
          double enu_x = msg->pose.pose.position.x;  // ENU 东（实际对应前）
          double enu_y = msg->pose.pose.position.y;  // ENU 北（实际对应左）
          double enu_z = msg->pose.pose.position.z;  // ENU 上

          // ENU → NED 映射：x→y, y→x, z→-z
          ned_pose_.header = msg->header;
          ned_pose_.header.frame_id = "map_ned";
          ned_pose_.pose.position.x = enu_y;   // NED x = ENU y（北）
          ned_pose_.pose.position.y = enu_x;   // NED y = ENU x（东）
          ned_pose_.pose.position.z = -enu_z;  // NED z = -ENU z（下）

          // 姿态直接透传，不转换
          ned_pose_.pose.orientation = msg->pose.pose.orientation;

          ned_pub_->publish(ned_pose_);
        });

    // 发布到 /vehicle_local_position（字段名对应 NED x/y/z）
    ned_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/vehicle_local_position", 10);

    RCLCPP_INFO(this->get_logger(), 
      "NED Position Listener started. ENU->NED: x->y, y->x, z->-z");
    RCLCPP_INFO(this->get_logger(), 
      "Subscribing /Odometry, Publishing /vehicle_local_position");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ned_pub_;
  geometry_msgs::msg::PoseStamped ned_pose_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NEDPositionListener>());
  rclcpp::shutdown();
  return 0;
}
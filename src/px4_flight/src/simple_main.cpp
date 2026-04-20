#include <rclcpp/rclcpp.hpp>
#include "px4_flight/px4_interface.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<px4_flight::PX4Interface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

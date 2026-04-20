// ~/px4_mid360_ws/src/px4_flight/src/livox_converter.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LivoxConverter : public rclcpp::Node {
public:
    LivoxConverter() : Node("livox_converter") {
        this->declare_parameter("input_topic", "/livox/lidar");
        this->declare_parameter("output_topic", "/livox/lidar_converted");
        this->declare_parameter("use_ring", true);  // Mid360S使用line->ring
        
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Livox Converter: %s -> %s (VELO16 format)", 
                    input_topic.c_str(), output_topic.c_str());

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, 
            std::bind(&LivoxConverter::callback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 检查输入字段
        bool has_line = false;
        for (const auto& field : msg->fields) {
            if (field.name == "line") has_line = true;
        }
        
        auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
        output->header = msg->header;
        output->height = msg->height;
        output->width = msg->width;
        output->is_dense = msg->is_dense;
        output->is_bigendian = msg->is_bigendian;
        
        // VELO16格式: x, y, z, intensity, ring, time (22 bytes)
        sensor_msgs::PointCloud2Modifier modifier(*output);
        modifier.setPointCloud2Fields(6,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "ring", 1, sensor_msgs::msg::PointField::UINT16,
            "time", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        modifier.resize(msg->width * msg->height);
        
        // 迭代器
        sensor_msgs::PointCloud2Iterator<float> out_x(*output, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*output, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*output, "z");
        sensor_msgs::PointCloud2Iterator<float> out_i(*output, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> out_ring(*output, "ring");
        sensor_msgs::PointCloud2Iterator<float> out_time(*output, "time");
        
        // 输入迭代器
        sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> in_i(*msg, "intensity");
        
        // 获取帧起始时间
        // double base_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        float frame_duration = 0.1f;  // Mid360S: 10Hz = 100ms
        
        // 可选：line字段
        sensor_msgs::PointCloud2ConstIterator<uint8_t> in_line(*msg, "line");
        
        size_t count = 0;
        size_t total_points = msg->width * msg->height;
        
        for (; count < total_points; 
             ++count, ++in_x, ++in_y, ++in_z, ++in_i,
             ++out_x, ++out_y, ++out_z, ++out_i, ++out_ring, ++out_time) {
            
            *out_x = *in_x;
            *out_y = *in_y;
            *out_z = *in_z;
            *out_i = *in_i;
            
            // line -> ring (Mid360S: 4线，0-3)
            if (has_line) {
                *out_ring = static_cast<uint16_t>(*in_line);
                ++in_line;
            } else {
                *out_ring = 0;  // 默认
            }
            
            // 估计相对时间戳（均匀分布假设）
            // 更精确的方法：使用点的实际采集时间（如果雷达提供）
            *out_time = (static_cast<float>(count) / total_points - 0.5f) * frame_duration;
        }
        
        pub_->publish(*output);
        
        static int log_count = 0;
        if (++log_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Converted %zu points to VELO16 format", count);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxConverter>());
    rclcpp::shutdown();
    return 0;
}
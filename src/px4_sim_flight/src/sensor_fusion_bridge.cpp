#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cstring>

class SensorFusionBridge : public rclcpp::Node {
public:
    SensorFusionBridge() : Node("sensor_fusion_bridge") {
        
        pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", 10);
        
        // 已同步为 manual_gz_bridge 发布的话题 /x500_0/lidar/points
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/x500_0/lidar/points", 10,
            std::bind(&SensorFusionBridge::lidarCallback, this, std::placeholders::_1));
            
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/x500_0/imu/data", 10,
            std::bind(&SensorFusionBridge::imuCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Bridge initialized");
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        bool has_x = false, has_y = false, has_z = false, has_intensity = false;
        for (const auto& field : msg->fields) {
            if (field.name == "x") has_x = true;
            if (field.name == "y") has_y = true;
            if (field.name == "z") has_z = true;
            if (field.name == "intensity") has_intensity = true;
        }
        
        if (!has_x || !has_y || !has_z) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "PointCloud missing xyz fields");
            return;
        }

        sensor_msgs::msg::PointCloud2 out;
        out.header = msg->header;
        out.header.frame_id = "livox_frame";
        out.height = msg->height;
        out.width = msg->width;
        out.is_bigendian = msg->is_bigendian;
        out.point_step = 28;
        out.row_step = out.point_step * out.width;
        out.is_dense = msg->is_dense;
        
        out.fields.resize(6);
        out.fields[0].name = "x"; out.fields[0].offset = 0; 
        out.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; out.fields[0].count = 1;
        out.fields[1].name = "y"; out.fields[1].offset = 4; 
        out.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; out.fields[1].count = 1;
        out.fields[2].name = "z"; out.fields[2].offset = 8; 
        out.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; out.fields[2].count = 1;
        out.fields[3].name = "intensity"; out.fields[3].offset = 12; 
        out.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32; out.fields[3].count = 1;
        out.fields[4].name = "time"; out.fields[4].offset = 16; 
        out.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT64; out.fields[4].count = 1;
        out.fields[5].name = "ring"; out.fields[5].offset = 24; 
        out.fields[5].datatype = sensor_msgs::msg::PointField::UINT16; out.fields[5].count = 1;
        
        out.data.resize(out.row_step * out.height);
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        
        std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_intensity;
        if (has_intensity) {
            iter_intensity = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*msg, "intensity");
        }
        
        size_t offset = 0;
        double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
        uint16_t ring = 0;
        uint16_t pad16 = 0;
        
        for (size_t i = 0; i < msg->width * msg->height; ++i) {
            memcpy(&out.data[offset], &(*iter_x), sizeof(float));
            memcpy(&out.data[offset+4], &(*iter_y), sizeof(float));
            memcpy(&out.data[offset+8], &(*iter_z), sizeof(float));
            
            float intensity = has_intensity ? **iter_intensity : 0.0f;
            memcpy(&out.data[offset+12], &intensity, sizeof(float));
            
            memcpy(&out.data[offset+16], &stamp_sec, sizeof(double));
            memcpy(&out.data[offset+24], &ring, sizeof(uint16_t));
            memcpy(&out.data[offset+26], &pad16, sizeof(uint16_t));
            
            offset += out.point_step;
            ++iter_x; ++iter_y; ++iter_z;
            if (has_intensity) ++(*iter_intensity);
        }
        
        pub_lidar_->publish(out);
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        sensor_msgs::msg::Imu out = *msg;
        out.header.frame_id = "livox_imu";
        pub_imu_->publish(out);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusionBridge>());
    rclcpp::shutdown();
    return 0;
}

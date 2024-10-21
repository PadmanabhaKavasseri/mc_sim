#include "lidar_node.hpp"

LidarPublisher::LidarPublisher()
: Node("lidar_publisher"), distribution_(0.0, 10.0)
{
    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz
        std::bind(&LidarPublisher::publish_lidar_data, this)
    );
}

void LidarPublisher::publish_lidar_data()
{
    auto message = sensor_msgs::msg::LaserScan();
    message.header.stamp = this->now();
    message.header.frame_id = "lidar_link";

    // Set the range and angle parameters
    message.angle_min = -1.57;  // -90 degrees
    message.angle_max = 1.57;   // 90 degrees
    message.angle_increment = 0.01;  // 0.01 radian increments
    message.time_increment = 0.0;
    message.scan_time = 0.1;  // 10 Hz
    message.range_min = 0.1;
    message.range_max = 10.0;

    // Generate random ranges
    int num_readings = static_cast<int>((message.angle_max - message.angle_min) / message.angle_increment);
    message.ranges.resize(num_readings);
    for (int i = 0; i < num_readings; ++i) {
        message.ranges[i] = distribution_(generator_);
    }

    lidar_publisher_->publish(message);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPublisher>());
    rclcpp::shutdown();
    return 0;
}

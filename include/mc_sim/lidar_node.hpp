#ifndef LIDAR_PUBLISHER_HPP_
#define LIDAR_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <random>

class LidarPublisher : public rclcpp::Node
{
public:
    LidarPublisher();

private:
    void publish_lidar_data();
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::default_random_engine generator_;
    std::uniform_real_distribution<float> distribution_;
};

#endif // LIDAR_PUBLISHER_HPP_

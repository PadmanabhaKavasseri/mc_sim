#ifndef NAVIGATION_NODE_HPP_
#define NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode();
    ~NavigationNode();

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_publisher_;

    std::ofstream csv_file_;
    int lidar_processing_time_ms_;
    int imu_processing_time_ms_;
    int camera_processing_time_ms_;
};

#endif // NAVIGATION_NODE_HPP_

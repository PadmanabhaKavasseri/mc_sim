#ifndef EXECUTOR_HPP
#define EXECUTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class CustomExecutor : public rclcpp::Executor
{
public:
    CustomExecutor();
    void add_node(const rclcpp::Node::SharedPtr &node);
    void spin() override;

private:
    void process_camera_images();
    void process_other_topics();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    sensor_msgs::msg::Image::SharedPtr last_camera_image_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

#endif // EXECUTOR_HPP

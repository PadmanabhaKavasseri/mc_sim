#ifndef CAMERA_PUBLISHER_HPP_
#define CAMERA_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <random>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher();

private:
    void publish_camera_data();
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::default_random_engine generator_;
    std::uniform_int_distribution<uint8_t> distribution_;
};

#endif // CAMERA_PUBLISHER_HPP_

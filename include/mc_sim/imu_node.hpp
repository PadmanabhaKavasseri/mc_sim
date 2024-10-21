#ifndef IMU_PUBLISHER_HPP_
#define IMU_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <random>

class ImuPublisher : public rclcpp::Node
{
public:
    ImuPublisher();

private:
    void publish_imu_data();
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
};

#endif // IMU_PUBLISHER_HPP_

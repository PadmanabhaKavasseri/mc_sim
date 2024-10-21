#include "imu_node.hpp"

ImuPublisher::ImuPublisher()
: Node("imu_publisher"), distribution_(0.0, 1.0)
{
    this->declare_parameter<int>("frequency", 10);
    this->get_parameter("frequency", frequency_);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / frequency_),
        std::bind(&ImuPublisher::publish_imu_data, this)
    );
}

void ImuPublisher::publish_imu_data()
{
    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->now();
    message.header.frame_id = "imu_link";

    // Generate random IMU data
    message.orientation.x = distribution_(generator_);
    message.orientation.y = distribution_(generator_);
    message.orientation.z = distribution_(generator_);
    message.orientation.w = distribution_(generator_);

    message.angular_velocity.x = distribution_(generator_);
    message.angular_velocity.y = distribution_(generator_);
    message.angular_velocity.z = distribution_(generator_);

    message.linear_acceleration.x = distribution_(generator_);
    message.linear_acceleration.y = distribution_(generator_);
    message.linear_acceleration.z = distribution_(generator_);

    imu_publisher_->publish(message);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPublisher>());
    rclcpp::shutdown();
    return 0;
}

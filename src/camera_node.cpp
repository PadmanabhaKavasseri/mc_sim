#include "camera_node.hpp"

CameraPublisher::CameraPublisher()
: Node("camera_publisher"), distribution_(0, 255)
{
    camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz
        std::bind(&CameraPublisher::publish_camera_data, this)
    );
}

void CameraPublisher::publish_camera_data()
{
    auto message = sensor_msgs::msg::Image();
    message.header.stamp = this->now();
    message.header.frame_id = "camera_link";
    message.height = 480;  // Example height
    message.width = 640;   // Example width
    message.encoding = "rgb8";
    message.is_bigendian = false;
    message.step = message.width * 3;  // 3 bytes per pixel for RGB

    // Generate random image data
    message.data.resize(message.height * message.step);
    for (auto & pixel : message.data) {
        pixel = distribution_(generator_);
    }

    camera_publisher_->publish(message);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto node = std::make_shared<CameraPublisher>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}

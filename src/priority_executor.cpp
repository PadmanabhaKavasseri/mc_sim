#include "priority_executor.hpp"

CustomExecutor::CustomExecutor()
    : rclcpp::Executor(rclcpp::ExecutorOptions())
{
}

void CustomExecutor::add_node(const rclcpp::Node::SharedPtr &node)
{
    camera_image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/images", 10,
        this {
            last_camera_image_ = msg;
        });

    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        this {
            last_scan_ = msg;
        });

    rclcpp::Executor::add_node(node);
}

void CustomExecutor::spin()
{
    while (rclcpp::ok())
    {
        if (last_camera_image_)
        {
            process_camera_images();
            last_camera_image_.reset();
        }
        else
        {
            process_other_topics();
        }

        rclcpp::spin_some(this->shared_from_this());
    }
}

void CustomExecutor::process_camera_images()
{
    RCLCPP_INFO(this->get_logger(), "Processing camera images");
    // Add your image processing code here
}

void CustomExecutor::process_other_topics()
{
    if (last_scan_)
    {
        RCLCPP_INFO(this->get_logger(), "Processing scan data");
        // Add your scan processing code here
        last_scan_.reset();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No prioritized topics, processing other topics");
        // Add code to process other topics here
    }
}

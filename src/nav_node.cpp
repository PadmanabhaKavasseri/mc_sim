#include "nav_node.hpp"
#include "priority_executor.hpp"

NavigationNode::NavigationNode()
: Node("navigation_node")
{
    this->declare_parameter<int>("lidar_processing_time_ms", 50);
    this->declare_parameter<int>("imu_processing_time_ms", 50);
    this->declare_parameter<int>("camera_processing_time_ms", 50);

    this->get_parameter("lidar_processing_time_ms", lidar_processing_time_ms_);
    this->get_parameter("imu_processing_time_ms", imu_processing_time_ms_);
    this->get_parameter("camera_processing_time_ms", camera_processing_time_ms_);

    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&NavigationNode::lidar_callback, this, std::placeholders::_1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, std::bind(&NavigationNode::imu_callback, this, std::placeholders::_1));
    camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image", 10, std::bind(&NavigationNode::camera_callback, this, std::placeholders::_1));
    navigation_publisher_ = this->create_publisher<std_msgs::msg::String>("navigation/output", 10);

    // Open CSV file for writing
    csv_file_.open("delays.csv", std::ios::out | std::ios::trunc);
    csv_file_ << "timestamp,lidar_delay,imu_delay,camera_delay\n";
}

NavigationNode::~NavigationNode()
{
    if (csv_file_.is_open()) {
        csv_file_.close();
    }
}

void NavigationNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    auto now = this->now();
    auto delay = now - msg->header.stamp;
    RCLCPP_INFO(this->get_logger(), "LiDAR delay: %f seconds", delay.seconds());

    // Simulate processing time
    std::this_thread::sleep_for(std::chrono::milliseconds(lidar_processing_time_ms_));

    // Write delay to CSV
    csv_file_ << std::fixed << std::setprecision(9) << now.seconds() << "," << delay.seconds() << ",,\n";

    // Publish a message to the navigation topic
    auto nav_msg = std_msgs::msg::String();
    nav_msg.data = "Processed LiDAR data";
    navigation_publisher_->publish(nav_msg);
}

void NavigationNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    auto now = this->now();
    auto delay = now - msg->header.stamp;
    RCLCPP_INFO(this->get_logger(), "IMU delay: %f seconds", delay.seconds());

    // Simulate processing time
    std::this_thread::sleep_for(std::chrono::milliseconds(imu_processing_time_ms_));

    // Write delay to CSV
    csv_file_ << std::fixed << std::setprecision(9) << now.seconds() << ",," << delay.seconds() << ",\n";

    // Publish a message to the navigation topic
    auto nav_msg = std_msgs::msg::String();
    nav_msg.data = "Processed IMU data";
    navigation_publisher_->publish(nav_msg);
}

void NavigationNode::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto now = this->now();
    auto delay = now - msg->header.stamp;
    RCLCPP_INFO(this->get_logger(), "Camera delay: %f seconds", delay.seconds());

    // Simulate processing time
    std::this_thread::sleep_for(std::chrono::milliseconds(camera_processing_time_ms_));

    // Write delay to CSV
    csv_file_ << std::fixed << std::setprecision(9) << now.seconds() << ",,," << delay.seconds() << "\n";

    // Publish a message to the navigation topic
    auto nav_msg = std_msgs::msg::String();
    nav_msg.data = "Processed Camera data";
    navigation_publisher_->publish(nav_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();

    // auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto executor = std::make_shared<PriorityExecutor>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}

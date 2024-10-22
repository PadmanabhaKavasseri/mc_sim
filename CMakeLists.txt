cmake_minimum_required(VERSION 3.8)
project(mc_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(./include/mc_sim)

# LiDAR Node
add_executable(lidar_node src/lidar_node.cpp)
ament_target_dependencies(lidar_node rclcpp sensor_msgs)

# IMU Node
add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs)

# Camera Node
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs)

# Navigation Node
add_executable(navigation_node src/nav_node.cpp)
ament_target_dependencies(navigation_node rclcpp sensor_msgs nav_msgs)

# Multi-threaded Navigation Node
add_executable(multi_nav_node src/multi_nav_node.cpp)
ament_target_dependencies(multi_nav_node rclcpp sensor_msgs std_msgs)

# Motor Controller Node
# add_executable(motor_controller_node src/motor_controller_node.cpp)
# ament_target_dependencies(motor_controller_node rclcpp std_msgs)



# Install targets
install(TARGETS
  lidar_node
  imu_node
  camera_node
  navigation_node
  multi_nav_node
  # motor_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()




# colcon build --packages-select mc_sim
# ros2 launch mc_sim launch.py
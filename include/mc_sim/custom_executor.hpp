#ifndef CUSTOM_EXECUTOR_HPP_
#define CUSTOM_EXECUTOR_HPP_

#include "rclcpp/rclcpp.hpp"

class CustomExecutor : public rclcpp::Executor
{
public:
  CustomExecutor(const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());
  ~CustomExecutor();

  void spin() override;

protected:
  void spin_once_impl(std::chrono::nanoseconds timeout) override;
};

#endif  // CUSTOM_EXECUTOR_HPP_

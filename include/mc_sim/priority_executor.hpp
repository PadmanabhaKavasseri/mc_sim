#ifndef PRIORITY_EXECUTOR_HPP
#define PRIORITY_EXECUTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include <queue>
#include <functional>

class PriorityExecutor : public rclcpp::Executor
{
public:
    PriorityExecutor();

    void spin_once_impl(std::chrono::nanoseconds timeout) override;
    void spin() override;

private:
    std::queue<rclcpp::AnyExecutable> camera_queue_;
    std::queue<rclcpp::AnyExecutable> other_queue_;

    bool get_next_executable(rclcpp::AnyExecutable & any_exec, std::chrono::nanoseconds timeout);
};

#endif // PRIORITY_EXECUTOR_HPP

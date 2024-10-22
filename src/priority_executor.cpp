#include "priority_executor.hpp"

PriorityExecutor::PriorityExecutor()
: rclcpp::Executor(rclcpp::ExecutorOptions())
{
}

void PriorityExecutor::spin_once_impl(std::chrono::nanoseconds timeout)
{
    rclcpp::AnyExecutable any_exec;
    if (!camera_queue_.empty()) {
        any_exec = camera_queue_.front();
        camera_queue_.pop();
    } else if (get_next_executable(any_exec, timeout)) {
        if (any_exec.subscription && any_exec.subscription->get_topic_name() == "camera/image") {
            camera_queue_.push(any_exec);
            return;
        }
    }

    if (any_exec.callback_group) {
        any_exec.callback_group->can_be_taken_from().store(false);
    }
    execute_any_executable(any_exec);
    if (any_exec.callback_group) {
        any_exec.callback_group->can_be_taken_from().store(true);
    }
}

void PriorityExecutor::spin()
{
    while (rclcpp::ok()) {
        spin_once_impl(std::chrono::nanoseconds::max());
    }
}

bool PriorityExecutor::get_next_executable(rclcpp::AnyExecutable & any_exec, std::chrono::nanoseconds timeout)
{
    return rclcpp::Executor::get_next_executable(any_exec, timeout);
}

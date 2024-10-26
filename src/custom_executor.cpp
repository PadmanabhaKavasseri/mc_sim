#include "custom_executor.hpp"

CustomExecutor::CustomExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options)
{
}

CustomExecutor::~CustomExecutor()
{
}

void CustomExecutor::spin()
{
  // Custom spin logic or call base class implementation
  while (rclcpp::ok()) {
    spin_once_impl(std::chrono::nanoseconds(100000000));  // Example timeout of 100ms
  }
}

void CustomExecutor::spin_once_impl(std::chrono::nanoseconds timeout)
{
  // Custom logic or call base class implementation
  rclcpp::Executor::spin_once(timeout);
}



/*

void CustomExecutor::spin_once_impl(std::chrono::nanoseconds timeout)
{
  // Create a wait set
  rclcpp::WaitSet wait_set;

  // Add subscriptions, services, timers, etc. to the wait set
  for (auto & handle : get_all_handles()) {
    wait_set.add_handle(handle);
  }

  // Wait for events
  auto result = wait_set.wait(timeout);

  // Check which handles are ready and create executables
  for (auto & handle : get_all_handles()) {
    if (wait_set.is_ready(handle)) {
      auto executable = create_executable(handle);
      execute_executable(executable);
    }
  }
}

std::shared_ptr<Executable> CustomExecutor::create_executable(rclcpp::AnyExecutable handle)
{
  // Create an executable from the handle
  auto executable = std::make_shared<Executable>();
  executable->callback = handle.get_callback();
  executable->data = handle.get_data();
  return executable;
}

void CustomExecutor::execute_executable(std::shared_ptr<Executable> executable)
{
  // Execute the callback
  executable->callback(executable->data);
}






*/
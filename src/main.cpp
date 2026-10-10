#include "dexh13_hand/dexh13_node.hpp"

#include <memory>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = dexh13_hand::createNode();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.cancel();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}

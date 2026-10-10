#pragma once

#include <memory>

#include <rclcpp/node.hpp>

namespace dexh13_hand
{

std::shared_ptr<rclcpp::Node> createNode();

}  // namespace dexh13_hand

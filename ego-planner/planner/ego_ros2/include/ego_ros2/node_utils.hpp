#pragma once
#include <algorithm>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
namespace ego_ros2
{
template<class T>
void parameter(const rclcpp::Node::SharedPtr &node, std::string name,
               T &value, const T &default_value)
{
  std::replace(name.begin(), name.end(), '/', '.');
  if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
  value = node->get_parameter(name).get_value<T>();
}
inline rclcpp::Clock::SharedPtr &clock()
{
  static auto value = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  return value;
}
inline void use_clock(const rclcpp::Node::SharedPtr &node) { clock() = node->get_clock(); }
inline rclcpp::Time now() { return clock()->now(); }
}

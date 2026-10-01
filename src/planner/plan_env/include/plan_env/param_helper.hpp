#ifndef _PLAN_ENV_PARAM_HELPER_HPP_
#define _PLAN_ENV_PARAM_HELPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace param_helper
{
  // ROS 1 nh.param(name, var, default) equivalent.
  // ROS 1 "a/b" names become ROS 2 "a.b" (nested YAML keys).
  // Declares the parameter once, so several modules may read the same name.
  // An integer value given for a double parameter is accepted, as in ROS 1.
  template <typename T>
  void param(const rclcpp::Node::SharedPtr &node, std::string name, T &var, const T &default_val)
  {
    for (auto &c : name)
      if (c == '/')
        c = '.';

    rclcpp::Parameter p;
    if (!node->has_parameter(name))
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.dynamic_typing = true;
      node->declare_parameter(name, rclcpp::ParameterValue(default_val), desc);
    }
    node->get_parameter(name, p);

    if constexpr (std::is_same_v<T, double>)
    {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        var = static_cast<double>(p.as_int());
        return;
      }
    }
    var = p.get_value<T>();
  }
} // namespace param_helper

#endif

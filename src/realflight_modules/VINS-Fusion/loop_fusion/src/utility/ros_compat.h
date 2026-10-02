/*******************************************************
 * ROS 2 shims for the ROS 1 logging/assert macros and time helpers used
 * throughout the VINS core, so the estimator code stays unchanged.
 *******************************************************/

#pragma once

#include <cmath>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

#define VINS_LOGGER rclcpp::get_logger("loop_fusion")

#define ROS_DEBUG(...) RCLCPP_DEBUG(VINS_LOGGER, __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(VINS_LOGGER, __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(VINS_LOGGER, __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(VINS_LOGGER, __VA_ARGS__)
#define ROS_DEBUG_STREAM(x) RCLCPP_DEBUG_STREAM(VINS_LOGGER, x)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(VINS_LOGGER, x)
#define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(VINS_LOGGER, x)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(VINS_LOGGER, x)

#define ROS_BREAK()                                                     \
    do {                                                                \
        RCLCPP_FATAL(VINS_LOGGER, "BREAKPOINT HIT\n\tfile = %s\n\tline=%d\n", \
                     __FILE__, __LINE__);                               \
        std::abort();                                                   \
    } while (0)

#define ROS_ASSERT(cond)                                                \
    do {                                                                \
        if (!(cond)) {                                                  \
            RCLCPP_FATAL(VINS_LOGGER, "ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n", \
                         __FILE__, __LINE__, #cond);                    \
            std::abort();                                               \
        }                                                               \
    } while (0)

// double seconds <-> builtin_interfaces/Time (replacements for ros::Time(t) / stamp.toSec())
inline builtin_interfaces::msg::Time toRosTime(double t)
{
    builtin_interfaces::msg::Time stamp;
    int64_t ns = static_cast<int64_t>(std::llround(t * 1e9));
    stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return stamp;
}

inline double toSec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

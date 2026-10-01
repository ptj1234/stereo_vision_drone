#ifndef __UAV_UTILS_H
#define __UAV_UTILS_H

#include <cstdio>
#include <cstdlib>

// Replacement for ROS 1 ROS_ASSERT_MSG: enabled unless NDEBUG is defined, prints and aborts.
#ifndef NDEBUG
#define UAV_UTILS_ASSERT_MSG(cond, ...)                                              \
  do                                                                                 \
  {                                                                                  \
    if (!(cond))                                                                     \
    {                                                                                \
      std::fprintf(stderr, "ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n\tmessage = ", \
                   __FILE__, __LINE__, #cond);                                       \
      std::fprintf(stderr, __VA_ARGS__);                                             \
      std::fprintf(stderr, "\n");                                                    \
      std::abort();                                                                  \
    }                                                                                \
  } while (0)
#else
#define UAV_UTILS_ASSERT_MSG(cond, ...) do { } while (0)
#endif

#include <uav_utils/converters.h>
#include <uav_utils/geometry_utils.h>

namespace uav_utils
{

/* judge if value belongs to [low,high] */
template <typename T, typename T2>
bool
in_range(T value, const T2& low, const T2& high)
{
  UAV_UTILS_ASSERT_MSG(low < high, "%f < %f?", low, high);
  return (low <= value) && (value <= high);
}

/* judge if value belongs to [-limit, limit] */
template <typename T, typename T2>
bool
in_range(T value, const T2& limit)
{
  UAV_UTILS_ASSERT_MSG(limit > 0, "%f > 0?", limit);
  return in_range(value, -limit, limit);
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& low, const T2& high)
{
  UAV_UTILS_ASSERT_MSG(low < high, "%f < %f?", low, high);
  if (value < low)
  {
    value = low;
  }

  if (value > high)
  {
    value = high;
  }

  return;
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& limit)
{
  UAV_UTILS_ASSERT_MSG(limit > 0, "%f > 0?", limit);
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
} // end of namespace uav_utils

#endif

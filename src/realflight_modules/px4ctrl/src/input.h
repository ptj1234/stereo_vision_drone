#ifndef __INPUT_H
#define __INPUT_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

// Single time source for px4ctrl (replaces ros::Time::now()).
// main() points it at the node clock; every stamp it creates is RCL_ROS_TIME,
// so subtracting any two px4ctrl times can never throw a clock-type mismatch.
namespace px4ctrl_time
{
  inline rclcpp::Clock::SharedPtr &clock()
  {
    static rclcpp::Clock::SharedPtr c = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    return c;
  }
  inline rclcpp::Time now() { return clock()->now(); }
  inline rclcpp::Time zero() { return rclcpp::Time(0, 0, RCL_ROS_TIME); }
}

class RC_Data_t
{
public:
  double mode;
  double gear;
  double reboot_cmd;
  double last_mode;
  double last_gear;
  double last_reboot_cmd;
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];

  mavros_msgs::msg::RCIn msg;
  rclcpp::Time rcv_stamp{px4ctrl_time::zero()};

  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;
  bool toggle_reboot;

  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
  static constexpr double DEAD_ZONE = 0.25;

  RC_Data_t();
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::msg::RCIn::ConstSharedPtr pMsg);
  bool is_received(const rclcpp::Time &now_time);
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
  Eigen::Vector3d w;

  nav_msgs::msg::Odometry msg;
  rclcpp::Time rcv_stamp{px4ctrl_time::zero()};
  bool recv_new_msg;

  Odom_Data_t();
  void feed(nav_msgs::msg::Odometry::ConstSharedPtr pMsg);
};

class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;

  sensor_msgs::msg::Imu msg;
  rclcpp::Time rcv_stamp{px4ctrl_time::zero()};

  Imu_Data_t();
  void feed(sensor_msgs::msg::Imu::ConstSharedPtr pMsg);
};

class State_Data_t
{
public:
  mavros_msgs::msg::State current_state;
  mavros_msgs::msg::State state_before_offboard;

  State_Data_t();
  void feed(mavros_msgs::msg::State::ConstSharedPtr pMsg);
};

class ExtendedState_Data_t
{
public:
  mavros_msgs::msg::ExtendedState current_extended_state;

  ExtendedState_Data_t();
  void feed(mavros_msgs::msg::ExtendedState::ConstSharedPtr pMsg);
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double yaw;
  double yaw_rate;

  quadrotor_msgs::msg::PositionCommand msg;
  rclcpp::Time rcv_stamp{px4ctrl_time::zero()};

  Command_Data_t();
  void feed(quadrotor_msgs::msg::PositionCommand::ConstSharedPtr pMsg);
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::msg::BatteryState msg;
  rclcpp::Time rcv_stamp{px4ctrl_time::zero()};

  Battery_Data_t();
  void feed(sensor_msgs::msg::BatteryState::ConstSharedPtr pMsg);
};

class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};
  uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination

  quadrotor_msgs::msg::TakeoffLand msg;
  rclcpp::Time rcv_stamp{px4ctrl_time::zero()};

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::msg::TakeoffLand::ConstSharedPtr pMsg);
};

#endif
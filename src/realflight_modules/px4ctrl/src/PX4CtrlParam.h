#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <string>

class Parameter_t
{
public:
	struct Gain
	{
		double Kp0, Kp1, Kp2;
		double Kv0, Kv1, Kv2;
		double Kvi0, Kvi1, Kvi2;
		double Kvd0, Kvd1, Kvd2;
		double KAngR, KAngP, KAngY;
	};

	struct RotorDrag
	{
		double x, y, z;
		double k_thrust_horz;
	};

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};

	Gain gain;
	RotorDrag rt_drag;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;

	int pose_solver;
	double mass;
	double gra;
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;

	bool use_bodyrate_ctrl;
	// bool print_dbg;

	Parameter_t();
	void config_from_ros_handle(const rclcpp::Node::SharedPtr &nh);
	void config_full_thrust(double hov);

private:
	// ROS 1 nh.getParam() + ROS_BREAK() equivalent: a missing parameter aborts the node.
	// ROS 1 "a/b" names are ROS 2 "a.b" (nested YAML keys).
	// An integer value given for a double parameter is accepted, as in ROS 1.
	template <typename TVal>
	void read_essential_param(const rclcpp::Node::SharedPtr &nh, std::string name, TVal &val)
	{
		for (auto &c : name)
			if (c == '/')
				c = '.';

		if (!nh->has_parameter(name))
		{
			rcl_interfaces::msg::ParameterDescriptor desc;
			desc.dynamic_typing = true;
			nh->declare_parameter(name, rclcpp::ParameterValue(), desc);
		}
		rclcpp::Parameter p = nh->get_parameter(name);

		bool ok = true;
		try
		{
			if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
				ok = false;
			else if constexpr (std::is_same_v<TVal, double>)
				val = (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) ? static_cast<double>(p.as_int()) : p.as_double();
			else
				val = p.get_value<TVal>();
		}
		catch (const rclcpp::ParameterTypeException &e)
		{
			RCLCPP_ERROR_STREAM(nh->get_logger(), "Param " << name << " has a wrong type: " << e.what());
			ok = false;
		}

		if (!ok)
		{
			RCLCPP_FATAL_STREAM(nh->get_logger(), "Read param: " << name << " failed.");
			std::abort();
		}
	};
};

#endif
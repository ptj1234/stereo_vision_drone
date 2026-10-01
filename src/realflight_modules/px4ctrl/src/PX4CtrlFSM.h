#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>

#include "input.h"
// #include "ThrustCurve.h"
#include "controller.h"

struct AutoTakeoffLand_t
{
	bool landed{true};
	rclcpp::Time toggle_takeoff_land_time{px4ctrl_time::zero()};
	std::pair<bool, rclcpp::Time> delay_trigger{std::pair<bool, rclcpp::Time>(false, px4ctrl_time::zero())};
	Eigen::Vector4d start_pose;
	
	static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motors idle running for 3 seconds before takeoff
	static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
};

class PX4CtrlFSM
{
public:
	Parameter_t &param;

	RC_Data_t rc_data;
	State_Data_t state_data;
	ExtendedState_Data_t extended_state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Command_Data_t cmd_data;
	Battery_Data_t bat_data;
	Takeoff_Land_Data_t takeoff_land_data;

	LinearControl &controller;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traj_start_trigger_pub;
	rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr ctrl_FCU_pub;
	rclcpp::Publisher<quadrotor_msgs::msg::Px4ctrlDebug>::SharedPtr debug_pub; //debug
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_FCU_mode_srv;
	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_srv;
	rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr reboot_FCU_srv;

	// ROS 1 ros::spinOnce(): runs pending subscription callbacks. Set by main().
	// process() runs in main(), outside any callback, so it may call this.
	std::function<void()> spin_once;
	// Executor that only serves the service clients' callback group, so waiting for a
	// service response does not run subscription callbacks (same as ROS 1 blocking call()).
	rclcpp::Executor::SharedPtr srv_executor;
	// Longer than mavros' command ACK timeout (5 s), so a slow ACK is not reported as a failure.
	static constexpr double SRV_CALL_TIMEOUT = 6.0;

	quadrotor_msgs::msg::Px4ctrlDebug debug_msg; //debug

	Eigen::Vector4d hover_pose;
	rclcpp::Time last_set_hover_pose_time{px4ctrl_time::zero()};

	enum State_t
	{
		MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER, // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// px4ctrl is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND
	};

	PX4CtrlFSM(Parameter_t &, LinearControl &);
	void process();
	bool rc_is_received(const rclcpp::Time &now_time);
	bool cmd_is_received(const rclcpp::Time &now_time);
	bool odom_is_received(const rclcpp::Time &now_time);
	bool imu_is_received(const rclcpp::Time &now_time);
	bool bat_is_received(const rclcpp::Time &now_time);
	bool recv_new_odom();
	State_t get_state() { return state; }
	bool get_landed() { return takeoff_land.landed; }

private:
	State_t state; // Should only be changed in PX4CtrlFSM::process() function!
	AutoTakeoffLand_t takeoff_land;

	// ---- control related ----
	Desired_State_t get_hover_des();
	Desired_State_t get_cmd_des();

	// ---- auto takeoff/land ----
	void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
	void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing 
	void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
	Desired_State_t get_rotor_speed_up_des(const rclcpp::Time now);
	Desired_State_t get_takeoff_land_des(const double speed);

	// ---- tools ----
	void set_hov_with_odom();
	void set_hov_with_rc();

	bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
	bool toggle_arm_disarm(bool arm); // It will only try to toggle once, so not blocked.
	void reboot_FCU();

	void publish_bodyrate_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp);
	void publish_trigger(const nav_msgs::msg::Odometry &odom_msg);

	// ROS 1 ServiceClient::call(): false if the service is not available or does not answer.
	template <typename SrvT>
	bool call_service(const typename rclcpp::Client<SrvT>::SharedPtr &client,
					  const typename SrvT::Request::SharedPtr &req,
					  typename SrvT::Response::SharedPtr &res)
	{
		if (!client->service_is_ready())
		{
			RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"), "Service %s is not available!", client->get_service_name());
			return false;
		}
		auto future = client->async_send_request(req);
		auto ret = srv_executor->spin_until_future_complete(
			future, std::chrono::duration<double>(SRV_CALL_TIMEOUT));
		if (ret != rclcpp::FutureReturnCode::SUCCESS)
		{
			client->remove_pending_request(future);
			RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"), "Service %s did not respond.", client->get_service_name());
			return false;
		}
		res = future.get();
		return true;
	}
};

#endif
/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "LocalCartesian.hpp"
#include "tic_toc.h"

// double seconds -> builtin_interfaces/Time (replacement for ros::Time(t))
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

using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
	nav_msgs::msg::Path global_path;

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void optimize();
	void updateGlobalPath();

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
	map<double, vector<double>> GPSPositionMap;
	bool initGPS;
	bool newGPS;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;

};
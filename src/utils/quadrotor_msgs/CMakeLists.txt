cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AuxCommand.msg
  msg/Corrections.msg
  msg/Gains.msg
  msg/GoalSet.msg
  msg/OutputData.msg
  msg/PositionCommand.msg
  msg/PPROutputData.msg
  msg/Serial.msg
  msg/SO3Command.msg
  msg/StatusData.msg
  msg/TRPYCommand.msg
  msg/Odometry.msg
  msg/PolynomialTrajectory.msg
  msg/OptimalTimeAllocator.msg
  msg/TrajectoryMatrix.msg
  msg/ReplanCheck.msg
  msg/SpatialTemporalTrajectory.msg
  msg/Bspline.msg
  msg/SwarmCommand.msg
  msg/Replan.msg
  msg/SwarmOdometry.msg
  msg/SwarmInfo.msg
  msg/Px4ctrlDebug.msg
  msg/TakeoffLand.msg
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

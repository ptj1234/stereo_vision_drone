cmake_minimum_required(VERSION 3.8)
project(px4ctrl)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)
target_include_directories(px4ctrl_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(px4ctrl_node
  rclcpp quadrotor_msgs geometry_msgs nav_msgs sensor_msgs mavros_msgs uav_utils
)

install(TARGETS px4ctrl_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

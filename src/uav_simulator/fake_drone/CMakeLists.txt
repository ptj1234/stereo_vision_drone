cmake_minimum_required(VERSION 3.8)
project(poscmd_2_odom)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(poscmd_2_odom src/poscmd_2_odom.cpp)
target_include_directories(poscmd_2_odom PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(poscmd_2_odom rclcpp nav_msgs quadrotor_msgs plan_env)

install(TARGETS poscmd_2_odom DESTINATION lib/${PROJECT_NAME})

ament_package()

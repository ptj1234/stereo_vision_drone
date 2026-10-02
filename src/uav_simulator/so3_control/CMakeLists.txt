cmake_minimum_required(VERSION 3.8)
project(so3_control)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(SO3Control src/SO3Control.cpp)
target_include_directories(SO3Control PUBLIC include ${EIGEN3_INCLUDE_DIRS})
set_target_properties(SO3Control PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ROS 1 nodelet so3_control/SO3ControlNodelet -> rclcpp component
add_library(so3_control_component SHARED src/so3_control_nodelet.cpp)
ament_target_dependencies(so3_control_component
  rclcpp rclcpp_components nav_msgs sensor_msgs std_msgs quadrotor_msgs tf2 tf2_geometry_msgs plan_env)
target_link_libraries(so3_control_component SO3Control)
rclcpp_components_register_node(so3_control_component
  PLUGIN "SO3ControlNodelet"
  EXECUTABLE so3_control_node)

add_executable(control_example src/control_example.cpp)
target_include_directories(control_example PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(control_example rclcpp quadrotor_msgs)

install(TARGETS so3_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS control_example DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

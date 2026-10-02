cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

## Qt signals/slots macros only, avoid clashes with boost signals
add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME} SHARED
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/goal_tool.h
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rviz_common rviz_rendering pluginlib geometry_msgs quadrotor_msgs)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

ament_package()

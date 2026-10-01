cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Bspline.msg
  msg/DataDisp.msg
  msg/MultiBsplines.msg
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

add_library(${PROJECT_NAME}_lib
  src/planning_visualization.cpp
  src/polynomial_traj.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp visualization_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_lib EXPORT export_${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime rclcpp visualization_msgs geometry_msgs Eigen3)
ament_package()

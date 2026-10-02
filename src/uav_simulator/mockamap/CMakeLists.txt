cmake_minimum_required(VERSION 3.8)
project(mockamap)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(plan_env REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(Eigen3 REQUIRED)

# src/ces_randommap.cpp is unused (no main, nothing references it) and stays ROS 1.
add_executable(${PROJECT_NAME}_node
  src/mockamap.cpp
  src/maps.cpp
  src/perlinnoise.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs pcl_conversions plan_env)
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

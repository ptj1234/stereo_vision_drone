cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)

# The CUDA depth renderer (src/pcl_render_node.cpp, src/depth_render.cu) is
# still ROS 1 code; only the CPU point cloud renderer is ported.
set(ENABLE_CUDA false)
if(ENABLE_CUDA)
  message(FATAL_ERROR "local_sensing_node: CUDA renderer not ported to ROS 2 yet")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(plan_env REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search)
find_package(Eigen3 REQUIRED)

add_executable(pcl_render_node src/pointcloud_render_node.cpp)
target_include_directories(pcl_render_node PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pcl_render_node
  rclcpp sensor_msgs geometry_msgs nav_msgs pcl_conversions plan_env)
target_link_libraries(pcl_render_node ${PCL_LIBRARIES})

install(TARGETS pcl_render_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_package()

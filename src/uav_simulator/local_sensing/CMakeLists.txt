cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)

# OFF: CPU renderer (src/pointcloud_render_node.cpp) -> point cloud ~/pcl_render_node/cloud
# ON:  CUDA renderer (src/pcl_render_node.cpp + src/depth_render.cu) -> depth image ~/depth,
#      ~/camera_pose, ~/colordepth, ~/rendered_pcl
# Both build an executable named pcl_render_node.
option(ENABLE_CUDA "Build the CUDA depth image renderer instead of the CPU point cloud renderer" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)

if(ENABLE_CUDA)
  if(NOT DEFINED CMAKE_CUDA_COMPILER AND EXISTS /usr/local/cuda/bin/nvcc)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
  endif()
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 87)  # Jetson Orin (ROS 1 build targeted sm_75)
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 -use_fast_math")

  find_package(cv_bridge REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common io)

  add_library(depth_render_cuda STATIC src/depth_render.cu)
  target_link_libraries(depth_render_cuda CUDA::cudart)

  add_executable(pcl_render_node src/pcl_render_node.cpp)
  target_include_directories(pcl_render_node PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(pcl_render_node
    rclcpp sensor_msgs geometry_msgs nav_msgs pcl_conversions plan_env cv_bridge)
  target_link_libraries(pcl_render_node depth_render_cuda CUDA::cudart ${OpenCV_LIBS} ${PCL_LIBRARIES})
else()
  find_package(PCL REQUIRED COMPONENTS common filters kdtree search)

  add_executable(pcl_render_node src/pointcloud_render_node.cpp)
  target_include_directories(pcl_render_node PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(pcl_render_node
    rclcpp sensor_msgs geometry_msgs nav_msgs pcl_conversions plan_env)
  target_link_libraries(pcl_render_node ${PCL_LIBRARIES})
endif()

install(TARGETS pcl_render_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_package()

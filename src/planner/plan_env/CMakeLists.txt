cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

set(dependencies
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  cv_bridge message_filters pcl_conversions
)

add_library(plan_env
  src/grid_map.cpp
  src/raycast.cpp
  src/obj_predictor.cpp
)
target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(plan_env ${dependencies})
target_link_libraries(plan_env ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(obj_generator src/obj_generator.cpp)
target_include_directories(obj_generator PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
ament_target_dependencies(obj_generator ${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS plan_env EXPORT export_plan_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS obj_generator DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} Eigen3 PCL OpenCV)
ament_package()

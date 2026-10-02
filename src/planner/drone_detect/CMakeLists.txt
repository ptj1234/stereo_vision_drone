cmake_minimum_required(VERSION 3.8)
project(drone_detect)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
  src/drone_detector.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs cv_bridge plan_env)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/drone_detector/ DESTINATION include/drone_detector FILES_MATCHING PATTERN "*.h")
install(DIRECTORY config launch doc DESTINATION share/${PROJECT_NAME})

ament_package()

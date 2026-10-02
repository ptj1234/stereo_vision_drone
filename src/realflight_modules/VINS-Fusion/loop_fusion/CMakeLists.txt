cmake_minimum_required(VERSION 3.8)
project(loop_fusion)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_models REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(loop_fusion_node
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    )
target_include_directories(loop_fusion_node PRIVATE ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(loop_fusion_node
  rclcpp ament_index_cpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  cv_bridge camera_models)
target_link_libraries(loop_fusion_node ${OpenCV_LIBS} Ceres::ceres)

install(TARGETS loop_fusion_node DESTINATION lib/${PROJECT_NAME})
install(FILES ../support_files/brief_k10L6.bin ../support_files/brief_pattern.yml
  DESTINATION share/${PROJECT_NAME}/support_files)

ament_package()

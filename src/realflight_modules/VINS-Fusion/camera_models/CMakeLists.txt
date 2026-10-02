cmake_minimum_required(VERSION 3.8)
project(camera_models)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

set(CAMERA_MODELS_SRCS
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

# shared (as under catkin) so the private Boost link stays out of the export
add_library(camera_models SHARED ${CAMERA_MODELS_SRCS})
target_include_directories(camera_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(camera_models PUBLIC ${OpenCV_LIBS} Ceres::ceres PRIVATE ${Boost_LIBRARIES})

add_executable(Calibrations src/intrinsic_calib.cc)
target_link_libraries(Calibrations camera_models ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS camera_models EXPORT export_camera_models
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS Calibrations DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_camera_models HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV Ceres Eigen3)
ament_package()

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (FinalProject_Camera src/matching2D_Student.cpp src/FinalProject_Camera.cpp src/lidarData.cpp src/objectDetection2D.cpp src/camFusion_Student.cpp)
target_link_libraries (FinalProject_Camera ${OpenCV_LIBRARIES})
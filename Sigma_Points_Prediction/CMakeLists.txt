cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(sigma_points_prediction)

find_package(OpenCV 4.1 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (sigma_points_prediction main.cpp ukf.cpp)

target_link_libraries (sigma_points_prediction ${OpenCV_LIBRARIES})
target_link_libraries (sigma_points_prediction Eigen3::Eigen)

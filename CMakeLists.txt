cmake_minimum_required(VERSION 3.10)
project(planning_algorithms)

find_package(OpenCV 4.4   REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(OpenCV_INCLUDE_DIRS)

message(STATUS "Opencv_include_Path: " ${OpenCV_INCLUDE_DIRS})
message(STATUS "Opencv_Library_Path: " ${OpenCV_LIBRARY})
# message(STATUS "Opencv_Library_Path: " ${OpenCV_LIBS})
message(STATUS "OpenCV_LIBRARY_DIRS: " ${OpenCV_LIBRARY_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})


add_library(rrt_lib rrt/rrt.cc)
target_link_libraries(rrt_lib ${OpenCV_LIBS} )
add_executable(rrt rrt/main.cpp)
target_link_libraries(rrt ${OpenCV_LIBS}  rrt_lib)


cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
project(get_distance_with_gnuplot)

include_directories(/usr/local/include/urg_cpp)
link_directories(/usr/local/lib)

find_package (OpenCV REQUIRED)

add_executable (get_distance_with_opencv get_distance_with_opencv.cpp Connection_information.cpp)
target_link_libraries(get_distance_with_opencv urg_cpp ${OpenCV_LIBRARIES})

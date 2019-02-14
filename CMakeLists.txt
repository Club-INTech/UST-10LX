cmake_minimum_required(VERSION 3.9)
project(LiDAR_UST_10LX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

add_executable(LiDAR_UST_10LX main.cpp DataPoint.h UST10LX.cpp UST10LX.h)
cmake_minimum_required(VERSION 3.9)
project(LiDAR_UST_10LX)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

add_executable(LiDAR_UST_10LX main.cpp DataPoint.h UST10LX.cpp UST10LX.h ObstacleFinder.cpp ObstacleFinder.h Client.cpp Client.h)
target_link_libraries(LiDAR_UST_10LX -static)
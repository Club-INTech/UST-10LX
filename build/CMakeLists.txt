cmake_minimum_required(VERSION 3.7)
project(LiDAR_UST_10LX)

set(CMAKE_CXX_STANDARD 11)
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

add_executable(LiDAR_UST_10LX ../src/main.cpp ../src/DataPoint.h ../src/UST10LX.cpp ../src/UST10LX.h ../src/ObstacleFinder.cpp ../src/ObstacleFinder.h ../src/Client.cpp ../src/Client.h)
target_link_libraries(LiDAR_UST_10LX -static)

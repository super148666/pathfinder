cmake_minimum_required(VERSION 2.8.3)
project(pathfinder)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

include_directories(
        include
        /usr/local/include
        /opt/X11/include
)
link_directories(
        /usr/local/lib
        /opt/X11/lib
)

set(SRC
        src/pathfinder.cpp
        include/pathfinder/pathfinder.h
        src/cone_2d.cpp
        include/pathfinder/cone_2d.h
        src/range_1d.cpp
        include/pathfinder/range_1d.h
        src/range_1d_cluster.cpp
        include/pathfinder/range_1d_cluster.h
        include/pathfinder/enum_def.h
        include/pathfinder/steering_range.h
        src/steering_range.cpp)

add_executable(demo src/main_demo.cpp ${SRC})
target_link_libraries(demo eyesim X11)

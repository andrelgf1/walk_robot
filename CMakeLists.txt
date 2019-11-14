cmake_minimum_required(VERSION 2.8.3)
project(walk_robot)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    geometry_msgs 
    move_base_msgs
    sensor_msgs
)


## Declare a catkin package
catkin_package()

# Build 
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walk_robot src/main.cpp src/walkRobot.cpp)
target_link_libraries(walk_robot ${catkin_LIBRARIES})

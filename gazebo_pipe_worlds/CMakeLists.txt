cmake_minimum_required(VERSION 3.0.2)
project(gazebo_pipe_worlds)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)


## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS
    gazebo_ros
)

include_directories(
    include)
include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

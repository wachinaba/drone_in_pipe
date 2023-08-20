cmake_minimum_required(VERSION 3.0.2)
project(localization_in_pipe)

find_package(catkin REQUIRED COMPONENTS
gazebo_ros_optical_flow_msgs  
geometry_msgs
  mavros
  mavros_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
# include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

message("catkin_LIBRARIES: ${catkin_LIBRARIES}")

add_executable(single_optical_flow_speed_estimator src/single_optical_flow_speed_estimator.cpp)
target_link_libraries(single_optical_flow_speed_estimator ${catkin_LIBRARIES})

add_executable(p3d_odometry_converter src/p3d_odometry_converter.cpp)
target_link_libraries(p3d_odometry_converter ${catkin_LIBRARIES})

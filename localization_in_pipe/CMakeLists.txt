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
  localization_in_pipe_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
  pluginlib
  nodelet
)


#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})


find_package(Eigen3 REQUIRED)
find_package(Kalman REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_ukf flow_range_integrator flow_range_integrator_node body_accel_estimator body_angular_velocity_estimator ${PROJECT_NAME}
  CATKIN_DEPENDS gazebo_ros_optical_flow_msgs geometry_msgs mavros mavros_msgs roscpp rospy sensor_msgs std_msgs visualization_msgs nav_msgs localization_in_pipe_msgs tf2 tf2_ros pcl_conversions pcl_ros nodelet pluginlib
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${KALMAN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

message("catkin_LIBRARIES: ${catkin_LIBRARIES} ")

add_library(move_ukf src/move_ukf.cpp)
target_link_libraries(move_ukf ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${KALMAN_LIBRARIES})
add_dependencies(move_ukf ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(flow_range_integrator src/flow_range_integrator.cpp)
target_link_libraries(flow_range_integrator ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${KALMAN_LIBRARIES})
add_dependencies(flow_range_integrator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(body_accel_estimator src/body_accel_estimator.cpp)
target_link_libraries(body_accel_estimator ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${KALMAN_LIBRARIES})
add_dependencies(body_accel_estimator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(body_angular_velocity_estimator src/body_angular_velocity_estimator.cpp)
target_link_libraries(body_angular_velocity_estimator ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${KALMAN_LIBRARIES})
add_dependencies(body_angular_velocity_estimator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(flow_range_integrator_node src/nodes/flow_range_integrator_node.cpp)
target_link_libraries(flow_range_integrator_node flow_range_integrator ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${KALMAN_LIBRARIES})
add_dependencies(flow_range_integrator_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}
  src/nodelets/flow_range_integrator_nodelet.cpp
  src/nodelets/move_ukf_nodelet.cpp
  src/nodelets/body_accel_estimator_nodelet.cpp
  src/nodelets/body_angular_velocity_estimator_nodelet.cpp
  src/nodelets/pointcloud1_to_2_converter_nodelet.cpp
  src/nodelets/pointcloud_merger_nodelet.cpp
  src/nodelets/pointcloud_transform_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME} move_ukf flow_range_integrator flow_range_integrator_node body_accel_estimator body_angular_velocity_estimator ${catkin_LIBRARIES})

add_executable(single_optical_flow_speed_estimator src/single_optical_flow_speed_estimator.cpp)
target_link_libraries(single_optical_flow_speed_estimator ${catkin_LIBRARIES})
add_dependencies(single_optical_flow_speed_estimator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(p3d_odometry_converter src/p3d_odometry_converter.cpp)
target_link_libraries(p3d_odometry_converter ${catkin_LIBRARIES})
add_dependencies(p3d_odometry_converter ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(level_plane_tf_broadcaster src/level_plane_tf_broadcaster.cpp)
target_link_libraries(level_plane_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(level_plane_tf_broadcaster ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(ransac_cylinder_estimator src/ransac_cylinder_estimator.cpp)
target_link_libraries(ransac_cylinder_estimator ${catkin_LIBRARIES})
add_dependencies(ransac_cylinder_estimator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#
# Install
#

install(TARGETS move_ukf flow_range_integrator flow_range_integrator_node body_accel_estimator body_angular_velocity_estimator ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(TARGETS single_optical_flow_speed_estimator p3d_odometry_converter level_plane_tf_broadcaster
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
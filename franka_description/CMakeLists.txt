cmake_minimum_required(VERSION 2.8.3)
project(franka_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	pluginlib
	pcl_conversions
	pcl_ros
	geometric_shapes
	gazebo_msgs
	gazebo_grasp_plugin
	moveit_visual_tools
  roscpp
  rospy
  roslint
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  PickPlace.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

catkin_package(CATKIN_DEPENDS xacro)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_executable(franka_pick_place_server src/franka_pick_place_server.cpp)
add_dependencies(franka_pick_place_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(franka_pick_place_server ${catkin_LIBRARIES})

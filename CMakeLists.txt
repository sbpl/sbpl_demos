cmake_minimum_required(VERSION 2.8.3)
project(sbpl_demos)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  moveit_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  RoconMoveArm.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs   geometry_msgs  moveit_msgs   sensor_msgs  std_msgs
)

###########
## Build ##
###########

catkin_package()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure



add_executable(pr2_rocon_marker_publisher src/pr2_rocon_marker_publisher.cpp)
add_dependencies(pr2_rocon_marker_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pr2_rocon_marker_publisher ${catkin_LIBRARIES} )

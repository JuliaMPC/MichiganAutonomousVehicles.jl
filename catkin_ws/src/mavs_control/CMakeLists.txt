
cmake_minimum_required(VERSION 2.8.3)
project(mavs_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  genmsg
  message_generation
  geometry_msgs
  obstacle_detector
  controller_manager
  joint_state_controller
  controller_interface
  joint_state_controller
  robot_state_publisher
  rqt_gui
  effort_controllers
)


## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Control.msg)
add_service_files(FILES GetObstacleData.srv)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Turn into a Python module using setup.py
catkin_python_setup()

## Generate added messages and services with any dependencies
generate_messages(DEPENDENCIES obstacle_detector std_msgs)

catkin_package()

cmake_minimum_required(VERSION 2.8.3)
project(mavs_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  geometry_msgs
  obstacle_detector
)

add_service_files(
  FILES
  GetObstacleData.srv
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Turn into a Python module using setup.py
catkin_python_setup()

## Generate added messages and services with any dependencies
generate_messages(DEPENDENCIES
                  obstacle_detector)

catkin_package()

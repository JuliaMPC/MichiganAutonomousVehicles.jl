cmake_minimum_required(VERSION 2.8.3)
project(mavs_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  gazebo_ros
)
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_velodyne_laser
  CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros
  DEPENDS gazebo roscpp gazebo_ros 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(model_push SHARED src/model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS gazebo_ros_velodyne_laser
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(mavs_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
  gazebo_ros
  gazebo
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_velodyne_laser
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs gazebo_ros
  DEPENDS gazebo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
# For Gazebo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

install(TARGETS gazebo_ros_velodyne_laser
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Build our plugin
add_library(velodyne_plugin SHARED src/velodyne_plugin.cc)
target_link_libraries(velodyne_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

# Build the stand-alone test program
add_executable(vel src/vel.cc)
target_link_libraries(vel ${GAZEBO_LIBRARIES})

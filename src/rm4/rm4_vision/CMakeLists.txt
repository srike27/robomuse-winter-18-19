cmake_minimum_required(VERSION 2.8.3)
project(rm4_vision)
find_package(catkin REQUIRED COMPONENTS roscpp rospy actionlib std_msgs actionlib_msgs genmsg urdf tf nav_msgs message_generation
             pluginlib
             cmake_modules
             geometric_shapes
	           sensor_msgs
             )

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
    interactive_markers
    actionlib_msgs
)
find_package(Eigen REQUIRED)

# include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(squarerm4 src/squarerm4.cpp)
target_link_libraries(squarerm4 ${catkin_LIBRARIES})



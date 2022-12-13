find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

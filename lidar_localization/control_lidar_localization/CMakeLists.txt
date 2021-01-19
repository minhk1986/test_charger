cmake_minimum_required(VERSION 3.0.2)
project(control_lidar_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sick_lidar_localization
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_lidar_localization
#  CATKIN_DEPENDS roscpp rospy sdt_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_lidar_localization src/control_lidar_localization.cpp)
add_dependencies(control_lidar_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_lidar_localization ${catkin_LIBRARIES})

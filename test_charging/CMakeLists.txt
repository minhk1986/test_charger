cmake_minimum_required(VERSION 3.0.2)
project(test_charging)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
  agv_define
  agv_main
  #sick_lidar_localization
)
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    message_runtime
    actionlib_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(charger_client src/charger_client.cpp)
add_dependencies(charger_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(charger_client ${catkin_LIBRARIES})

add_executable(charger_server src/charger_server.cpp)
add_dependencies(charger_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(charger_server ${catkin_LIBRARIES})
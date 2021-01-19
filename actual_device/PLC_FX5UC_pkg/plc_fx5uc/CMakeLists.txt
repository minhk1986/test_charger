cmake_minimum_required(VERSION 3.0.2)
project(plc_fx5uc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  diagnostic_msgs
  message_generation
  mbtcp
  fx5u_hardware
  agv_define
)

add_message_files(
   FILES
   fx5uc_IO.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plc_fx5uc
   CATKIN_DEPENDS roscpp std_msgs
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

add_executable(fx5uc_controller src/fx5uc_controller.cpp)
add_dependencies(fx5uc_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fx5uc_controller	${catkin_LIBRARIES})

include_directories(
# include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)


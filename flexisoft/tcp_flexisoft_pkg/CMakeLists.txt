cmake_minimum_required(VERSION 3.0.2)
project(tcp_flexisoft_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  agv_define
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tcp_flexisoft_pkg
  CATKIN_DEPENDS roscpp std_msgs message_runtime agv_define
#  DEPENDS system_lib
)

include_directories(
  include/tcp_flexisoft_pkg
  ${catkin_INCLUDE_DIRS}
)

add_library(flexisoft src/flexisoft.cpp)

#add_executable(fx_node src/fx_node.cpp)
#add_dependencies(fx_node ${fx_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(fx_node ${catkin_LIBRARIES} flexisoft)

add_executable(ipc_to_fx src/ipc_to_fx.cpp)
add_dependencies(ipc_to_fx ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ipc_to_fx ${catkin_LIBRARIES} flexisoft)


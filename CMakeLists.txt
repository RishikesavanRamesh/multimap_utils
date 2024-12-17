cmake_minimum_required(VERSION 3.0.2)
project(multimap_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
  genmsg
)

add_action_files(
  DIRECTORY action
  FILES MultiMapGoal.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(multimap_utils src/simple_goal.cpp)
target_link_libraries(multimap_utils ${catkin_LIBRARIES})

add_executable(change_map src/change_map.cpp)
target_link_libraries(change_map ${catkin_LIBRARIES})

add_executable(mm_action_client src/mm_action_client.cpp)
target_link_libraries(mm_action_client ${catkin_LIBRARIES})


add_executable(multimap_navigator src/multimap_navigator.cpp)
target_link_libraries(multimap_navigator ${catkin_LIBRARIES})

# add_executable(mm_action_server src/mm_action_server.cpp)
# target_link_libraries(mm_action_server ${catkin_LIBRARIES})
# add_dependencies(mm_action_server multimap_utils_generate_messages_cpp)
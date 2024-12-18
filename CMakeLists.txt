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
find_package(SQLite3 REQUIRED)

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
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


add_executable(steer_to_goal src/MultiMapGoalActionClient.cpp)
target_link_libraries(steer_to_goal ${catkin_LIBRARIES})





add_executable(multimap_navigator src/MultiMapGoalActionServer.cpp)
target_link_libraries(multimap_navigator ${catkin_LIBRARIES})
target_link_libraries(multimap_navigator ${SQLite3_LIBRARIES})


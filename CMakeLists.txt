cmake_minimum_required(VERSION 3.0.2)
project(multimap_utils)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
  std_srvs
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
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime geometry_msgs std_srvs
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


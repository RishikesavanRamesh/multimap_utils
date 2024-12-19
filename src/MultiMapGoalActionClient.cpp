#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "multimap_utils/MultiMapGoalAction.h"
#include <sstream>
#include <iostream>

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "mm_client");

  // Ensure that enough arguments are passed (3 arguments + 1 for the program name)
  if (argc != 4)
  {
    ROS_ERROR("Usage: mybin <x_pose> <y_pose> <map_name>");
    return 1;  // Exit with an error code
  }

  // Parse the command-line arguments
  float x_pose = std::stof(argv[1]);  // Convert the first argument to float (x position)
  float y_pose = std::stof(argv[2]);  // Convert the second argument to float (y position)
  std::string map_name = argv[3];     // Third argument is the map name (string)

  // Create the action client
  actionlib::SimpleActionClient<multimap_utils::MultiMapGoalAction> ac("multimap_goal_action");

  ROS_INFO("Waiting for action server to start.");
  // Wait for the action server to start
  ac.waitForServer(); // Will wait indefinitely for the action server

  ROS_INFO("Action server started, sending goal.");
  // Send a goal to the action
  multimap_utils::MultiMapGoalGoal goal;

  goal.target_pose.header.frame_id = "map"; 
  goal.target_pose.header.stamp = ros::Time::now();

  // Set the parsed pose values
  goal.target_pose.pose.position.x = x_pose;
  goal.target_pose.pose.position.y = y_pose;
  goal.target_pose.pose.position.z = 0.0;

  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  // Set the map name
  goal.map_name = map_name;

  // Send the goal to the action server
  ac.sendGoal(goal);

  // Wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the timeout.");
    ac.cancelGoal();
  }

  // Exit
  return 0;
}

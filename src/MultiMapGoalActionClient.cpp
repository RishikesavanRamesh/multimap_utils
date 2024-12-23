/*
Name: MultiMapGoalActionClient.cpp
Author: Rishikesavan Ramesh <automationwith.rishikesavan@gmail.com>
Date: 20/12/2024
Version: 1.0
Description:
    This file contains the client code that interacts with the MultiMapGoalAction server to send a goal.
    The client allows sending a navigation goal to a multi-map action server. The user provides the
    desired target pose (x, y coordinates) and the map name as command-line arguments, and the client
    sends the goal to the action server. The client waits for the action to complete, and upon success,
    prints the result state.

    The client works by:
    - Parsing input from the command line (x, y coordinates and map name).
    - Setting up an action client to communicate with the MultiMapGoalAction server.
    - Sending a goal to the server with a target pose and map name.
    - Waiting for the action server to process the goal and return a result.
    - Printing the state of the action once completed or handling timeouts.

Usage:
    This file should be compiled and executed as a client for the MultiMapGoalAction server.
    The user should provide the target pose (x, y) and the map name as arguments.

    Example usage:
    $ rosrun your_package mm_client 1.0 2.0 map_1

*/


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iostream>
#include <multimap_utils/MultiMapGoalAction.h>
#include <ros/ros.h>
#include <sstream>

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
  float xPose = std::stof(argv[1]);  // Convert the first argument to float (x position)
  float yPose = std::stof(argv[2]);  // Convert the second argument to float (y position)
  std::string mapName = argv[3];     // Third argument is the map name (string)

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
  goal.target_pose.pose.position.x = xPose;
  goal.target_pose.pose.position.y = yPose;
  goal.target_pose.pose.position.z = 0.0;

  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  // Set the map name
  goal.map_name = mapName;

  // Send the goal to the action server
  ac.sendGoal(goal);

  // Wait for the action to return
  bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));

  if (finishedBeforeTimeout)
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

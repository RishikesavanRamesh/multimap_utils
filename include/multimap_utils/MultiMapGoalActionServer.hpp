/*
 * Name: MultiMapGoalActionServer.hpp
 * Author: Rishikesavan Ramesh <automationwith.rishikesavan@gmail.com>
 * Date: 20/12/2024
 * Version: 1.0
 * Description: 
 *   This header file defines the MultiMapGoalActionServer class, which implements 
 *   an action server that handles map switching, localization, and navigation 
 *   between multiple maps using wormhole points. The class provides a mechanism 
 *   to switch between different maps, localize the robot in a given map, 
 *   navigate to wormhole points, and manage goals for navigation.
 *
 *   The core functionality includes receiving and processing goals, interacting 
 *   with the `move_base` action server for navigation, and managing multiple 
 *   maps with localization functionality.
 */

#ifndef MULTIMAP_GOAL_ACTION_SERVER_HPP
#define MULTIMAP_GOAL_ACTION_SERVER_HPP

#include <actionlib/client/service_client.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <chrono>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <multimap_utils/MultiMapGoalAction.h>
#include <nav_msgs/LoadMap.h>
#include <ros/ros.h>
#include <sqlite3.h>
#include <std_srvs/Empty.h>
#include <string>

class MultiMapGoalActionServer
{
public:
    /**
     * @brief Constructor for the MultiMapGoalActionServer class
     * @param name Name of the action
     */
    explicit MultiMapGoalActionServer(std::string name);

    /**
     * @brief Destructor for the MultiMapGoalActionServer class
     */
    ~MultiMapGoalActionServer();

    /**
     * @brief Callback function that processes the goal
     * @param goal The goal message passed to the action server
     */
    void executeCB(const actionlib::SimpleActionServer<multimap_utils::MultiMapGoalAction>::GoalConstPtr &goal);

private:
    /**
     * @brief Function to localize the robot at a given pose
     * @param wormholePose Pose to localize the robot at
     */
    void localize(const geometry_msgs::PoseStamped &wormholePose);

    /**
     * @brief Function to get the current map
     * @param requestedMap Name of the requested map
     * @return The current map name
     */
    std::string getCurrentMap(const std::string &requestedMap);

    /**
     * @brief Function to set the current map
     * @param requestedMap Name of the requested map
     */
    void setCurrentMap(const std::string &requestedMap);

    /**
     * @brief Function to get the wormhole points between two maps
     * @param currentMap Name of the current map
     * @param goalMap Name of the goal map
     * @return Pair of geometry_msgs::PoseStamped representing wormhole positions in both maps
     */
    std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> getWormholePoints(const std::string &currentMap, const std::string &goalMap);

    /**
     * @brief Function to print the pose details
     * @param pose Pose to be printed
     */
    void printPose(const geometry_msgs::PoseStamped &pose);

    /**
     * @brief Function to navigate the robot to the wormhole
     * @param wormhole Pose to navigate to
     */
    void navigateToWormhole(const geometry_msgs::PoseStamped &wormhole);

    /**
     * @brief Function to change the map to the target map
     * @param targetMap The map to switch to
     */
    void changeMap(const std::string &targetMap);

    /**
     * @brief Function to send the goal to move_base
     * @param goal Pose goal to be sent to move_base
     */
    void sendGoalToMoveBase(const geometry_msgs::PoseStamped &goal);

private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<multimap_utils::MultiMapGoalAction> multiMapGoalServer;
    std::string m_currentMap; // Stores the current map name
};

#endif // MULTIMAP_GOAL_ACTION_SERVER_HPP

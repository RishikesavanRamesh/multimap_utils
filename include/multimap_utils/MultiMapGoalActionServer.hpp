#ifndef MULTIMAP_GOAL_ACTION_SERVER_HPP
#define MULTIMAP_GOAL_ACTION_SERVER_HPP

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "multimap_utils/MultiMapGoalAction.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/LoadMap.h"
#include <sqlite3.h>
#include <string>
#include <actionlib/client/service_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
     * @param wormhole_pose Pose to localize the robot at
     */
    void localize(const geometry_msgs::PoseStamped &wormhole_pose);

    /**
     * @brief Function to get the current map
     * @param requested_map Name of the requested map
     * @return The current map name
     */
    std::string getCurrentMap(const std::string &requested_map);

    /**
     * @brief Function to set the current map
     * @param requested_map Name of the requested map
     */
    void setCurrentMap(const std::string &requested_map);

    /**
     * @brief Function to get the wormhole points between two maps
     * @param current_map Name of the current map
     * @param goal_map Name of the goal map
     * @return Pair of geometry_msgs::PoseStamped representing wormhole positions in both maps
     */
    std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> getWormholePoints(const std::string &current_map, const std::string &goal_map);

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
     * @param target_map The map to switch to
     */
    void changeMap(const std::string &target_map);

    /**
     * @brief Function to send the goal to move_base
     * @param goal Pose goal to be sent to move_base
     */
    void sendGoalToMoveBase(const geometry_msgs::PoseStamped &goal);

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<multimap_utils::MultiMapGoalAction> as_;
    std::string action_name_;
    std::string current_map_; // Stores the current map name
};

#endif // MULTIMAP_GOAL_ACTION_SERVER_HPP

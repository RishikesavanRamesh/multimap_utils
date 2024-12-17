#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <postgresql/libpq-fe.h> // For PostgreSQL integration (dummy for now)
#include "multimap_utils/MultiMapGoal.h"

// Define the custom action server
class MapNavigationActionServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::MultiMapActionServer<move_base_msgs::MoveBaseAction> as_; // Action server
    std::string action_name_;
    move_base_msgs::MoveBaseGoal goal_; // The goal to send to MoveBase
    move_base_msgs::MoveBaseResult result_;

public:
    MapNavigationActionServer(std::string name) : as_(nh_, name, boost::bind(&MapNavigationActionServer::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }

    ~MapNavigationActionServer() {}

    void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
    {
        ROS_INFO("%s: Executing, processing goal", action_name_.c_str());

        // The custom interface has a PoseStamped and map_name
        geometry_msgs::PoseStamped target_pose = goal->target_pose;
        std::string requested_map = goal->target_pose.header.frame_id; // Map name is passed in the header.frame_id

        std::string current_map = getCurrentMap(); // Get current map name

        if (current_map == requested_map)
        {
            ROS_INFO("Current map is the same as goal map. Sending goal directly.");
            sendGoalToMoveBase(target_pose); // Send goal directly to move_base
        }
        else
        {
            ROS_INFO("Current map is different. Changing maps...");

            // Get the wormhole coordinates (simulating database query)
            geometry_msgs::PoseStamped wormhole = getWormholePoint(current_map, requested_map); 
            navigateToWormhole(wormhole); // Navigate to wormhole point

            changeMap(requested_map); // Change to the requested map

            // Localize at the wormhole location on the new map and send the goal again
            ROS_INFO("Now localizing at wormhole location on the new map...");
            sendGoalToMoveBase(wormhole); // Localize at the wormhole position
            sendGoalToMoveBase(target_pose); // Send the final goal
        }

        as_.setSucceeded(result_);
    }

    std::string getCurrentMap()
    {
        // This should return the current map name. Placeholder function for now.
        return "current_map"; // Dummy value
    }

    geometry_msgs::PoseStamped getWormholePoint(std::string current_map, std::string goal_map)
    {
        // Dummy function to simulate fetching wormhole point from a database
        // Replace this with PostgreSQL queries to fetch actual wormhole points
        geometry_msgs::PoseStamped wormhole_pose;
        wormhole_pose.pose.position.x = 2.0; // Dummy x
        wormhole_pose.pose.position.y = 3.0; // Dummy y
        wormhole_pose.pose.orientation.w = 1.0; // Dummy orientation
        return wormhole_pose;
    }

    void navigateToWormhole(const geometry_msgs::PoseStamped &wormhole)
    {
        // Send the wormhole goal to move_base
        ROS_INFO("Navigating to wormhole point...");
        sendGoalToMoveBase(wormhole);
    }

    void changeMap(const std::string &new_map)
    {
        // Simulate changing the map (this would normally involve reloading the map)
        ROS_INFO("Changing to map: %s", new_map.c_str());
        // Here you could call a service to load the new map or reconfigure the map server
    }

    void sendGoalToMoveBase(const geometry_msgs::PoseStamped &goal)
    {
        // Create an action client for MoveBase
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        ROS_INFO("Waiting for move_base action server...");
        ac.waitForServer();

        // Send the goal to MoveBase
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose = goal;
        ac.sendGoal(move_base_goal);

        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
        if (finished_before_timeout)
        {
            ROS_INFO("Goal reached!");
        }
        else
        {
            ROS_ERROR("Failed to reach goal");
        }
    }


    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_map_navigation_action_server");

    MapNavigationActionServer server("multi_map_navigation_action");

    return 0;
}

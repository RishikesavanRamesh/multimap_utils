#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "multimap_utils/MultiMapGoalAction.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/LoadMap.h"
#include <sqlite3.h>
#include <string>

class MultiMapGoalActionServer
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<multimap_utils::MultiMapGoalAction> as_;
  std::string action_name_;
  std::string current_map; 

public:
  MultiMapGoalActionServer(std::string name)
    : as_(nh_, name, boost::bind(&MultiMapGoalActionServer::executeCB, this, _1), false),
      action_name_(name)
  {
    as_.start();
  }

  ~MultiMapGoalActionServer(void)
  {
  }

  void executeCB(const actionlib::SimpleActionServer<multimap_utils::MultiMapGoalAction>::GoalConstPtr &goal)
    {
        ROS_INFO("%s: Executing, processing goal", action_name_.c_str());

        // The custom interface has a PoseStamped and map_name
        geometry_msgs::PoseStamped target_pose = goal->target_pose;
        std::string requested_map = goal->map_name; // Map name is passed in the header.frame_id

        std::string current_map = getCurrentMap(requested_map); // Get current map name

        ROS_INFO("Current map: %s", current_map.c_str());  // Print current map
        ROS_INFO("Requested map: %s", requested_map.c_str());   // Print requested map

        if (current_map == requested_map)
        {
            ROS_INFO("Current map is the same as goal map. Sending goal directly.");
            printPose(target_pose);
            sendGoalToMoveBase(target_pose); // Send goal directly to move_base
        }
        else
        {
            ROS_INFO("Current map is different. Changing maps...");

            // Get the wormhole coordinates (simulating database query)
            geometry_msgs::PoseStamped wormhole = getWormholePoint(current_map, requested_map); 
            // geometry_msgs::PoseStamped wormhole_current_map, wormhole_requested_map = getWormholePoint(current_map, requested_map); 

            navigateToWormhole(wormhole); // Navigate to wormhole point

            changeMap(requested_map); // Change to the requested map
            setCurrentMap(requested_map);
            // Localize at the wormhole location on the new map and send the goal again
            ROS_INFO("Now localizing at wormhole location on the new map...");
            sendGoalToMoveBase(wormhole); // Localize at the wormhole position
            sendGoalToMoveBase(target_pose); // Send the final goal
        }

        as_.setSucceeded();
    }

    // Function to get the current map
    std::string getCurrentMap(const std::string& requested_map)
    {
        // If the current map is empty, set it to the requested map
        if (current_map.empty())
        {
            current_map = requested_map;
            ROS_INFO("Setting current map to: %s", current_map.c_str());
        }
        return current_map;
    }

    // Function to set the current map
    void setCurrentMap(const std::string& requested_map)
    {
        current_map = requested_map;
    }

    geometry_msgs::PoseStamped getWormholePointHC(std::string current_map, std::string goal_map)
    {
        geometry_msgs::PoseStamped wormhole_pose;

          wormhole_pose.header.frame_id = "map"; 
          wormhole_pose.header.stamp = ros::Time::now();

        // Hardcoded wormhole logic based on maps
        if (current_map == "map_part1" && goal_map == "map_part2")
        {
            // If current map is map_part1 and requested map is map_part2
            wormhole_pose.pose.position.x = 8.0;
            wormhole_pose.pose.position.y = -5.0;
            wormhole_pose.pose.position.z = 0.0;

            wormhole_pose.pose.orientation.x = 0.0;  // Assume no rotation
            wormhole_pose.pose.orientation.y = 0.0;  // Assume no rotation
            wormhole_pose.pose.orientation.z = 0.0;  // Assume no rotation
            wormhole_pose.pose.orientation.w = 1.0;  // Assume no rotation
        }
        else if (current_map == "map_part2" && goal_map == "map_part1")
        {
            // If current map is map_part1 and requested map is map_part2
            wormhole_pose.pose.position.x = 8.0;
            wormhole_pose.pose.position.y = -5.0;
            wormhole_pose.pose.position.z = 0.0;

            wormhole_pose.pose.orientation.x = 0.0;  // Assume no rotation
            wormhole_pose.pose.orientation.y = 0.0;  // Assume no rotation
            wormhole_pose.pose.orientation.z = 0.0;  // Assume no rotation
            wormhole_pose.pose.orientation.w = 1.0;  // Assume no rotation
        }
        else
        {//TODO: rk : if no wormhole found cancel goal.. ass moving to 0, 0 makes it move to map frame
            // Default case when no matching wormhole is found
            ROS_WARN("No wormhole mapping found for the given maps.");
            // If current map is map_part1 and requested map is map_part2
            wormhole_pose.pose.position.x = 8.0;
            wormhole_pose.pose.position.y = -5.0;
            wormhole_pose.pose.position.z = 0.0;

            wormhole_pose.pose.orientation.x = 0.0;  // Assume no rotation
            wormhole_pose.pose.orientation.y = 0.0;  // Assume no rotation
            wormhole_pose.pose.orientation.z = 0.0;  // Assume no rotation
            wormhole_pose.pose.orientation.w = 1.0;  // Assume no rotation
        }

        // Return the wormhole pose
        return wormhole_pose;
    }
    

    geometry_msgs::PoseStamped getWormholePoint(const std::string& current_map, const std::string& goal_map)
    {
        sqlite3* db;
        sqlite3_stmt* stmt;
        geometry_msgs::PoseStamped wormhole_pose;


        wormhole_pose.header.frame_id = "map"; 
        wormhole_pose.header.stamp = ros::Time::now();

        // Open the SQLite database
        int rc = sqlite3_open("/home/developer/my_ros_ws/wormhole_graph.sqlite3", &db);
        if (rc) {
            ROS_ERROR("Can't open database: %s", sqlite3_errmsg(db));
            return wormhole_pose;  // Return empty pose
        }

        // Query to find the wormhole that exists in both maps
        std::string query = 
            "SELECT w.position_x, w.position_y, w.position_z, w.orientation_x, "
            "w.orientation_y, w.orientation_z, w.orientation_w "
            "FROM wormholes w "
            "JOIN map_wormhole_relations mw1 ON w.id = mw1.wormhole_id "
            "JOIN map_wormhole_relations mw2 ON w.id = mw2.wormhole_id "
            "JOIN maps m1 ON mw1.map_id = m1.id "
            "JOIN maps m2 ON mw2.map_id = m2.id "
            "WHERE m1.name = ? AND m2.name = ?;";

        // Prepare the SQL statement
        rc = sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, 0);
        if (rc) {
            ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db));
            sqlite3_close(db);
            return wormhole_pose;
        }

        // Bind the parameters (map names)
        sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, goal_map.c_str(), -1, SQLITE_STATIC);

        // Execute the query and check if we have a result
        rc = sqlite3_step(stmt);
        if (rc == SQLITE_ROW) {
            // Extract the values from the result
            wormhole_pose.pose.position.x = sqlite3_column_double(stmt, 0);
            wormhole_pose.pose.position.y = sqlite3_column_double(stmt, 1);
            wormhole_pose.pose.position.z = sqlite3_column_double(stmt, 2);
            wormhole_pose.pose.orientation.x = sqlite3_column_double(stmt, 3);
            wormhole_pose.pose.orientation.y = sqlite3_column_double(stmt, 4);
            wormhole_pose.pose.orientation.z = sqlite3_column_double(stmt, 5);
            wormhole_pose.pose.orientation.w = sqlite3_column_double(stmt, 6);
        } else {
            ROS_WARN("No wormhole mapping found for the given maps.");
            // Set default pose if no wormhole is found
            wormhole_pose.pose.position.x = 0.0;
            wormhole_pose.pose.position.y = 0.0;
            wormhole_pose.pose.position.z = 0.0;
            wormhole_pose.pose.orientation.x = 0.0;
            wormhole_pose.pose.orientation.y = 0.0;
            wormhole_pose.pose.orientation.z = 0.0;
            wormhole_pose.pose.orientation.w = 1.0;
        }

        // Clean up
        sqlite3_finalize(stmt);
        sqlite3_close(db);

        printPose(wormhole_pose);
        // Return the wormhole pose
        return wormhole_pose;
    }

    void printPose(const geometry_msgs::PoseStamped& pose)
    {
        ROS_INFO("Pose: ");
        ROS_INFO("FRAME_ID:\n");

        ROS_INFO("Frame_ -> x: %s", pose.header.frame_id.c_str());
        ROS_INFO("Position -> x: %f, y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        ROS_INFO("Orientation -> x: %f, y: %f, z: %f, w: %f", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    }

    void navigateToWormhole(const geometry_msgs::PoseStamped &wormhole)
    {
        // Send the wormhole goal to move_base
        ROS_INFO("Navigating to wormhole point...");
        printPose(wormhole);
        sendGoalToMoveBase(wormhole);
    }

    void changeMap(const std::string &target_map)
    {

        // Simulate changing the map (this would normally involve reloading the map)
        ROS_INFO("Changing to map: %s", target_map.c_str());
        
        ros::ServiceClient client = nh_.serviceClient<nav_msgs::LoadMap>("change_map");
        nav_msgs::LoadMap srv;std::string map_path = "/home/developer/my_ros_ws/src/AR100/anscer_navigation/maps/" + target_map + ".yaml";
        srv.request.map_url = map_path;
        if (client.call(srv))
        {
          ROS_INFO("Sum: %ld", (long int)srv.response.result);
        }
        else
        {
          ROS_ERROR("Failed to call service ");
        }


        // Here you could call a service to load the new map or reconfigure the map server
    }

    void localize()

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

        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          ROS_INFO("success");
        else
          ROS_INFO("The base failed to move forward");

    }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mm_navigator_server");

  // Create the action server object and pass the name of the action
  MultiMapGoalActionServer server("mm_navigator");

  // Keep the server running
  ros::spin();

  return 0;
}

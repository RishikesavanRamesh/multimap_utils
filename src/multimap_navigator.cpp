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

            auto wormhole_poses = getWormholePoints("current_map_name", "requested_map_name");
            geometry_msgs::PoseStamped wormhole_pose_current_map = wormhole_poses.first;
            geometry_msgs::PoseStamped wormhole_pose_requested_map = wormhole_poses.second;

            navigateToWormhole(wormhole_pose_current_map); // Navigate to wormhole point

            changeMap(requested_map); // Change to the requested map
            setCurrentMap(requested_map);
            // Localize at the wormhole location on the new map and send the goal again
            ROS_INFO("Now localizing at wormhole location on the new map...");
            localize(wormhole_pose_requested_map); // Localize at the wormhole position
            sendGoalToMoveBase(target_pose); // Send the final goal
        }

        as_.setSucceeded();
    }

    void localize(const geometry_msgs::PoseStamped &wormhole_pose){

        ros::Publisher localize_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        
        // Create an initial PoseWithCovarianceStamped message
        geometry_msgs::PoseWithCovarianceStamped initialPose;

        // Set the header for initialPose (matching wormhole_pose header information)
        initialPose.header = wormhole_pose.header;

        // Map the position and orientation from wormhole_pose to initialPose
        initialPose.pose.pose.position = wormhole_pose.pose.position;
        initialPose.pose.pose.orientation = wormhole_pose.pose.orientation;

        // Set the covariance matrix based on your provided covariance array
        double covariance[36] = {
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787
        };

        // Assign the covariance values into the PoseWithCovarianceStamped message
        for (int i = 0; i < 36; i++) {
            initialPose.pose.covariance[i] = covariance[i];
        }

        // Publish the initialPose
        localize_pub.publish(initialPose);
        
        ROS_INFO("Published initial pose for localization");
        
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
    
    std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> getWormholePoints(const std::string& current_map, const std::string& goal_map)
{
    sqlite3* db;
    sqlite3_stmt* stmt;
    geometry_msgs::PoseStamped wormhole_current_map;
    geometry_msgs::PoseStamped wormhole_requested_map;

    wormhole_current_map.header.frame_id = "base_link"; 
    wormhole_current_map.header.stamp = ros::Time::now();

    wormhole_requested_map.header.frame_id = "base_link"; 
    wormhole_requested_map.header.stamp = ros::Time::now();
    
    // Open the SQLite database
    int rc = sqlite3_open("/home/developer/my_ros_ws/wormhole_graph.sqlite3", &db);
    if (rc) {
        ROS_ERROR("Can't open database: %s", sqlite3_errmsg(db));
        ROS_WARN("Moving to Wormhole failed");
        return std::make_pair(wormhole_current_map, wormhole_requested_map);  // Return empty poses if failed
    }

    // Query to find the common wormhole between both maps (First Query)
    std::string goal_map_pose_query = 
        "SELECT mw1.wormhole_id, mw1.pose_x, mw1.pose_y, mw1.pose_z, "
        "mw1.orientation_x, mw1.orientation_y, mw1.orientation_z, mw1.orientation_w "
        "FROM map_wormhole_relations mw2 "
        "JOIN maps m1 ON mw2.map_id = m1.map_id "
        "JOIN map_wormhole_relations mw1 ON mw1.wormhole_id = mw2.wormhole_id "
        "JOIN maps m2 ON mw1.map_id = m2.map_id "
        "WHERE m1.map_name = ? AND m2.map_name = ?; ";


    // Query to find the common wormhole between both maps (Second Query)
    std::string current_map_pose_query = 
        "SELECT mw2.wormhole_id, mw2.pose_x, mw2.pose_y, mw2.pose_z, "
        "mw2.orientation_x, mw2.orientation_y, mw2.orientation_z, mw2.orientation_w "
        "FROM map_wormhole_relations mw2 "
        "JOIN maps m1 ON mw2.map_id = m1.map_id "
        "JOIN map_wormhole_relations mw1 ON mw1.wormhole_id = mw2.wormhole_id "
        "JOIN maps m2 ON mw1.map_id = m2.map_id "
        "WHERE m1.map_name = ? AND m2.map_name = ?; ";


    // Prepare the SQL statement for querying the common wormhole (First Query)
    rc = sqlite3_prepare_v2(db, goal_map_pose_query.c_str(), -1, &stmt, 0);
    if (rc) {
        ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        ROS_WARN("Moving to Wormhole failed");
        return std::make_pair(wormhole_current_map, wormhole_requested_map);
    }

    // Bind the current and goal map parameters
    sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, goal_map.c_str(), -1, SQLITE_STATIC);

    // Execute the query and check if a common wormhole is found
    rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
        // Extract the wormhole ID and its pose (position and orientation)
        int wormhole_id = sqlite3_column_int(stmt, 0);
        double pose_x = sqlite3_column_double(stmt, 1);
        double pose_y = sqlite3_column_double(stmt, 2);
        double pose_z = sqlite3_column_double(stmt, 3);
        double orientation_x = sqlite3_column_double(stmt, 4);
        double orientation_y = sqlite3_column_double(stmt, 5);
        double orientation_z = sqlite3_column_double(stmt, 6);
        double orientation_w = sqlite3_column_double(stmt, 7);

        // Set the position and orientation in the current map pose
        wormhole_requested_map.pose.position.x = pose_x;
        wormhole_requested_map.pose.position.y = pose_y;
        wormhole_requested_map.pose.position.z = pose_z;
        wormhole_requested_map.pose.orientation.x = orientation_x;
        wormhole_requested_map.pose.orientation.y = orientation_y;
        wormhole_requested_map.pose.orientation.z = orientation_z;
        wormhole_requested_map.pose.orientation.w = orientation_w;

        wormhole_requested_map.header.frame_id = "map";  // Set the frame_id for consistency

        // Print the common wormhole information from the first query
        ROS_INFO("Common wormhole found with ID: %d", wormhole_id);
        ROS_INFO("Position in goal_map: (%.2f, %.2f, %.2f)", pose_x, pose_y, pose_z);
        ROS_INFO("Orientation in goal_map: (%.2f, %.2f, %.2f, %.2f)", orientation_x, orientation_y, orientation_z, orientation_w);
    } else {
        ROS_WARN("No common wormhole found between maps: %s and %s", current_map.c_str(), goal_map.c_str());
    }

    // Clean up the first prepared statement
    sqlite3_finalize(stmt);

    // Prepare the SQL statement for querying the common wormhole (Second Query)
    rc = sqlite3_prepare_v2(db, current_map_pose_query.c_str(), -1, &stmt, 0);
    if (rc) {
        ROS_ERROR("Failed to prepare second statement: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        ROS_WARN("Moving to Wormhole failed");
        return std::make_pair(wormhole_current_map, wormhole_requested_map);
    }

    // Bind the current and goal map parameters for the second query
    sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, goal_map.c_str(), -1, SQLITE_STATIC);

    // Execute the second query and check if a common wormhole is found
    rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
        // Extract the wormhole ID and its pose (position and orientation)
        int wormhole_id = sqlite3_column_int(stmt, 0);
        double pose_x = sqlite3_column_double(stmt, 1);
        double pose_y = sqlite3_column_double(stmt, 2);
        double pose_z = sqlite3_column_double(stmt, 3);
        double orientation_x = sqlite3_column_double(stmt, 4);
        double orientation_y = sqlite3_column_double(stmt, 5);
        double orientation_z = sqlite3_column_double(stmt, 6);
        double orientation_w = sqlite3_column_double(stmt, 7);

        // Set the position and orientation in the current map pose
        wormhole_current_map.pose.position.x = pose_x;
        wormhole_current_map.pose.position.y = pose_y;
        wormhole_current_map.pose.position.z = pose_z;
        wormhole_current_map.pose.orientation.x = orientation_x;
        wormhole_current_map.pose.orientation.y = orientation_y;
        wormhole_current_map.pose.orientation.z = orientation_z;
        wormhole_current_map.pose.orientation.w = orientation_w;

        wormhole_current_map.header.frame_id = "map";  // Set the frame_id for consistency

        // Print the common wormhole information from the second query
        ROS_INFO("Common wormhole found with ID: %d (second query)", wormhole_id);
        ROS_INFO("Position in current_map: (%.2f, %.2f, %.2f)", pose_x, pose_y, pose_z);
        ROS_INFO("Orientation in current_map: (%.2f, %.2f, %.2f, %.2f)", orientation_x, orientation_y, orientation_z, orientation_w);
    } else {
        ROS_WARN("No common wormhole found between maps: %s and %s (second query)", current_map.c_str(), goal_map.c_str());
    }

    // Clean up the prepared statement and close the database
    sqlite3_finalize(stmt);
    sqlite3_close(db);

    // Return the poses
    return std::make_pair(wormhole_current_map, wormhole_requested_map);
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
        ROS_INFO("Navigated to wormhole point...");
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

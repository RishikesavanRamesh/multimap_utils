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

int main(int argc, char **argv)
{
    sqlite3* db;
    sqlite3_stmt* stmt;
    // geometry_msgs::PoseStamped wormhole_current_map;
    // geometry_msgs::PoseStamped wormhole_requested_map;

    // wormhole_current_map.header.frame_id = "map"; 
    // // wormhole_current_map.header.stamp = ros::Time::now();
    
    // Open the SQLite database
    int rc = sqlite3_open("/home/developer/my_ros_ws/wormhole_graph.sqlite3", &db);
    if (rc) {
        ROS_ERROR("Can't open database: %s", sqlite3_errmsg(db));
        // return std::make_pair(wormhole_current_map, wormhole_requested_map);  // Return empty poses if failed
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
        // return std::make_pair(wormhole_current_map, wormhole_requested_map);
    }

    std::string current_map = "map_part1";
    std::string goal_map = "map_part2";
    // Bind the current and goal map parameters for the first query
    sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, goal_map.c_str(), -1, SQLITE_STATIC);

    // Execute the first query and check if a common wormhole is found
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
    }

    // Bind the current and goal map parameters for the second query
    sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, goal_map.c_str(), -1, SQLITE_STATIC);

    // Execute the second query and check if a common wormhole is found
    rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
        // Extract the wormhole ID and its pose (position and orientation) from the second query
        int wormhole_id = sqlite3_column_int(stmt, 0);
        double pose_x = sqlite3_column_double(stmt, 1);
        double pose_y = sqlite3_column_double(stmt, 2);
        double pose_z = sqlite3_column_double(stmt, 3);
        double orientation_x = sqlite3_column_double(stmt, 4);
        double orientation_y = sqlite3_column_double(stmt, 5);
        double orientation_z = sqlite3_column_double(stmt, 6);
        double orientation_w = sqlite3_column_double(stmt, 7);

        // Print the common wormhole information from the second query
        ROS_INFO("Common wormhole found with ID: %d (second query)", wormhole_id);
        ROS_INFO("Position in current_map: (%.2f, %.2f, %.2f)", pose_x, pose_y, pose_z);
        ROS_INFO("Orientation in current_map: (%.2f, %.2f, %.2f, %.2f)", orientation_x, orientation_y, orientation_z, orientation_w);
    } else {
        ROS_WARN("No common wormhole found between maps: %s and %s (second query)", current_map.c_str(), goal_map.c_str());
    }

    // Clean up the second prepared statement and close the database
    sqlite3_finalize(stmt);
    sqlite3_close(db);

    return 0;
}

/*
Name: MultiMapGoalActionServer.cpp
Author: Rishikesavan Ramesh
Date: 20/12/2024
Version: 1.0
Description:
    This file defines the implementation of the MultiMapGoalActionServer class, 
    which manages the action server for handling multi-map navigation goals. 
    The server interacts with ROS action servers, service clients, and the robot's 
    localization and navigation systems. The file includes the following functionality:
    - Handling goal requests and actions with the SimpleActionServer.
    - Localizing the robot at specific poses within the map system.
    - Managing map changes and navigating between maps using wormholes.
    - Communicating with move_base for robot navigation.

    The implementation facilitates navigating between different maps using wormhole 
    points, managing robot localization, and sending movement commands to the robot.

Usage:
    This source file should be compiled along with the corresponding header file (`MultiMapGoalActionServer.hpp`) 
    and be used in the robot’s navigation stack to enable multi-map goal processing and robot navigation.

*/

#include "multimap_utils/MultiMapGoalActionServer.hpp"

int main(int argc, char** argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "multimap_goal_action_server");

    // Create an instance of the action server
    MultiMapGoalActionServer actionServer("multimap_goal_action");

    // Keep the program running and processing requests
    ros::spin();

    return 0;
}

// Constructor
MultiMapGoalActionServer::MultiMapGoalActionServer(std::string name)
    : multiMapGoalServer(nh, name, boost::bind(&MultiMapGoalActionServer::executeCB, this, _1), false),
      m_currentMap("")
{
    // Start the action server
    multiMapGoalServer.start();
}

// Destructor
MultiMapGoalActionServer::~MultiMapGoalActionServer(void) {}

// Callback function when a goal is received
void MultiMapGoalActionServer::executeCB(const actionlib::SimpleActionServer<multimap_utils::MultiMapGoalAction>::GoalConstPtr &goal)
{
    geometry_msgs::PoseStamped targetPose = goal->target_pose;
    std::string requestedMap = goal->map_name;

    std::string m_currentMap = getCurrentMap(requestedMap);  // Get current map

    ROS_INFO("Current map: %s", m_currentMap.c_str());
    ROS_INFO("Requested map: %s", requestedMap.c_str());

    if (m_currentMap == requestedMap)
    {
        ROS_INFO("Current map is the same as goal map. Sending goal directly.");
        printPose(targetPose);
        sendGoalToMoveBase(targetPose);
    }
    else
    {
        ROS_INFO("Current map is different. Changing maps...");

        auto wormholePoses = getWormholePoints(m_currentMap, requestedMap);
        geometry_msgs::PoseStamped wormholePoseCurrentMap = wormholePoses.first;
        geometry_msgs::PoseStamped wormholePoseRequestedMap = wormholePoses.second;

        navigateToWormhole(wormholePoseCurrentMap);
        changeMap(requestedMap);
        setCurrentMap(requestedMap);

        ROS_INFO("Now localizing at wormhole location on the new map...");
        localize(wormholePoseRequestedMap);
        sendGoalToMoveBase(targetPose);
    }

    multiMapGoalServer.setSucceeded();
}

// Localize the robot at a specific pose
void MultiMapGoalActionServer::localize(const geometry_msgs::PoseStamped &wormholePose)
{
    ros::Publisher localizingPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    geometry_msgs::PoseWithCovarianceStamped initialPose;
    initialPose.header = wormholePose.header;
    initialPose.pose.pose.position = wormholePose.pose.position;
    initialPose.pose.pose.orientation = wormholePose.pose.orientation;

    double covariance[36] = {
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.250, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.250, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.25
    };

    for (int i = 0; i < 36; i++) {
        initialPose.pose.covariance[i] = covariance[i];
    }


    std_srvs::Empty srv;

    // Get the current time
    ros::Time startTime = ros::Time::now();

    // Loop for 4 seconds
    while (ros::Time::now() - startTime < ros::Duration(4.0)) {
        // Call the clearCostmap service

        ros::service::call("/move_base_node/clear_costmaps", srv);

        ros::service::call("/request_nomotion_update", srv);
        
        localizingPosePublisher.publish(initialPose);

        // Sleep to allow the loop to run at a reasonable rate
        ros::Duration(0.5).sleep();  // sleep for 0.5 seconds before the next iteration
    }

    // call nomotionupdate and clear costmap continuosly for 5 seconds
    ROS_INFO("Published initial pose for localization");
}

// Get the current map being used
std::string MultiMapGoalActionServer::getCurrentMap(const std::string& requestedMap)
{
    if (m_currentMap.empty())
    {
        m_currentMap = requestedMap;
        ROS_INFO("Setting current map to: %s", m_currentMap.c_str());
    }
    return m_currentMap;
}

// Set the current map being used
void MultiMapGoalActionServer::setCurrentMap(const std::string& requestedMap)
{
    m_currentMap = requestedMap;
}

// Print the pose
void MultiMapGoalActionServer::printPose(const geometry_msgs::PoseStamped& pose)
{
    ROS_INFO("Pose: ");
    ROS_INFO("Frame: %s", pose.header.frame_id.c_str());
    ROS_INFO("Position -> x: %f, y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    ROS_INFO("Orientation -> x: %f, y: %f, z: %f, w: %f", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
}

// Send the goal to move_base
void MultiMapGoalActionServer::sendGoalToMoveBase(const geometry_msgs::PoseStamped &goal)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();

    move_base_msgs::MoveBaseGoal moveBaseGoal;
    moveBaseGoal.target_pose = goal;
    ac.sendGoal(moveBaseGoal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Successfully moved to the goal!");
    }
    else
    {   
        printPose(moveBaseGoal.target_pose);
        ROS_INFO("Failed to move to the goal.");
    }
}

// Navigate to the wormhole
void MultiMapGoalActionServer::navigateToWormhole(const geometry_msgs::PoseStamped &wormhole)
{
    ROS_INFO("Navigating to wormhole...");
    printPose(wormhole);
    sendGoalToMoveBase(wormhole);
}

// Change the map being used by the robot
void MultiMapGoalActionServer::changeMap(const std::string &target_map)
{
    ros::ServiceClient client = nh.serviceClient<nav_msgs::LoadMap>("change_map");
    nav_msgs::LoadMap srv;
    std::string mapPath = "/home/developer/my_ros_ws/src/AR100/anscer_navigation/maps/" + target_map + ".yaml";
    srv.request.map_url = mapPath;

    if (client.call(srv))
    {
        ROS_INFO("Map change successful");
    }
    else
    {
        ROS_ERROR("Failed to change map");
    }
}

// Get wormhole points between two maps
std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MultiMapGoalActionServer::getWormholePoints(const std::string& m_currentMap, const std::string& goalMap)

{
    sqlite3* db;
    sqlite3_stmt* stmt;
    geometry_msgs::PoseStamped wormholeCurrentMap;
    geometry_msgs::PoseStamped wormholeRequestedMap;

    wormholeCurrentMap.header.frame_id = "base_link"; 
    wormholeCurrentMap.header.stamp = ros::Time::now();
    wormholeRequestedMap.header.frame_id = "base_link"; 
    wormholeRequestedMap.header.stamp = ros::Time::now();
    
    // Open the SQLite database
    int rc = sqlite3_open("/home/developer/my_ros_ws/wormhole_graph.sqlite3", &db);
    if (rc) {
        ROS_ERROR("Can't open database: %s", sqlite3_errmsg(db));
        return std::make_pair(wormholeCurrentMap, wormholeRequestedMap);  // Return empty poses if failed
    }

    // Query to find the common wormhole between both maps (First Query)
    std::string goalMapPoseQuery = 
        "SELECT mw1.wormhole_id, mw1.pose_x, mw1.pose_y, mw1.pose_z, "
        "mw1.orientation_x, mw1.orientation_y, mw1.orientation_z, mw1.orientation_w "
        "FROM map_wormhole_relations mw2 "
        "JOIN maps m1 ON mw2.map_id = m1.map_id "
        "JOIN map_wormhole_relations mw1 ON mw1.wormhole_id = mw2.wormhole_id "
        "JOIN maps m2 ON mw1.map_id = m2.map_id "
        "WHERE m1.map_name = ? AND m2.map_name = ?; ";

    // Prepare the SQL statement for querying the common wormhole (First Query)
    rc = sqlite3_prepare_v2(db, goalMapPoseQuery.c_str(), -1, &stmt, 0);
    if (rc) {
        ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return std::make_pair(wormholeCurrentMap, wormholeRequestedMap);
    }

    ROS_INFO("cm : %s; gm : %s", m_currentMap.c_str(), goalMap.c_str());
    // Bind the current and goal map parameters for the first query
    sqlite3_bind_text(stmt, 1, m_currentMap.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, goalMap.c_str(), -1, SQLITE_STATIC);

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

        // Set the position and orientation in the requested map pose
        wormholeRequestedMap.pose.position.x = pose_x;
        wormholeRequestedMap.pose.position.y = pose_y;
        wormholeRequestedMap.pose.position.z = pose_z;
        wormholeRequestedMap.pose.orientation.x = orientation_x;
        wormholeRequestedMap.pose.orientation.y = orientation_y;
        wormholeRequestedMap.pose.orientation.z = orientation_z;
        wormholeRequestedMap.pose.orientation.w = orientation_w;

        wormholeRequestedMap.header.frame_id = "map";  // Set the frame_id for consistency


        // Log the common wormhole information from the first query
        ROS_INFO("Common wormhole found with ID: %d", wormhole_id);
        ROS_INFO("Position in goalMap: (%.2f, %.2f, %.2f)", pose_x, pose_y, pose_z);
        ROS_INFO("Orientation in goalMap: (%.2f, %.2f, %.2f, %.2f)", orientation_x, orientation_y, orientation_z, orientation_w);
    } else if (rc == SQLITE_DONE) {
        ROS_WARN("No common wormhole found between maps: %s and %s", m_currentMap.c_str(), goalMap.c_str());
    } else {
        ROS_ERROR("Error executing query: %s", sqlite3_errmsg(db));
    }

    // Clean up the first prepared statement
    sqlite3_finalize(stmt);

    // Prepare the SQL statement for querying the common wormhole (Second Query)
    std::string currentMapPoseQuery = 
        "SELECT mw2.wormhole_id, mw2.pose_x, mw2.pose_y, mw2.pose_z, "
        "mw2.orientation_x, mw2.orientation_y, mw2.orientation_z, mw2.orientation_w "
        "FROM map_wormhole_relations mw2 "
        "JOIN maps m1 ON mw2.map_id = m1.map_id "
        "JOIN map_wormhole_relations mw1 ON mw1.wormhole_id = mw2.wormhole_id "
        "JOIN maps m2 ON mw1.map_id = m2.map_id "
        "WHERE m1.map_name = ? AND m2.map_name = ?; ";

    rc = sqlite3_prepare_v2(db, currentMapPoseQuery.c_str(), -1, &stmt, 0);
    if (rc) {
        ROS_ERROR("Failed to prepare second statement: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return std::make_pair(wormholeCurrentMap, wormholeRequestedMap);
    }

    // Bind the current and goal map parameters for the second query
    sqlite3_bind_text(stmt, 1, m_currentMap.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, goalMap.c_str(), -1, SQLITE_STATIC);

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
        wormholeCurrentMap.pose.position.x = pose_x;
        wormholeCurrentMap.pose.position.y = pose_y;
        wormholeCurrentMap.pose.position.z = pose_z;
        wormholeCurrentMap.pose.orientation.x = orientation_x;
        wormholeCurrentMap.pose.orientation.y = orientation_y;
        wormholeCurrentMap.pose.orientation.z = orientation_z;
        wormholeCurrentMap.pose.orientation.w = orientation_w;

        wormholeCurrentMap.header.frame_id = "map";  // Set the frame_id for consistency


        // Log the common wormhole information from the second query
        ROS_INFO("Common wormhole found with ID: %d (second query)", wormhole_id);
        ROS_INFO("Position in m_currentMap: (%.2f, %.2f, %.2f)", pose_x, pose_y, pose_z);
        ROS_INFO("Orientation in m_currentMap: (%.2f, %.2f, %.2f, %.2f)", orientation_x, orientation_y, orientation_z, orientation_w);
    } else if (rc == SQLITE_DONE) {
        ROS_WARN("No common wormhole found between maps: %s and %s (second query)", m_currentMap.c_str(), goalMap.c_str());
    } else {
        ROS_ERROR("Error executing second query: %s", sqlite3_errmsg(db));
    }

    // Clean up the second prepared statement and close the database
    sqlite3_finalize(stmt);
    sqlite3_close(db);
    ROS_WARN("NOW WILL PRINT REQ MAP WH POSE");
    printPose(wormholeRequestedMap);
    ROS_WARN("NOW WILL PRINT CUR MAP WH POSE");
    printPose(wormholeCurrentMap);

    return std::make_pair(wormholeCurrentMap, wormholeRequestedMap);
}


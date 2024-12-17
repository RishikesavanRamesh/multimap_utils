#include "ros/ros.h"
#include "nav_msgs/LoadMap.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_loader");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<nav_msgs::LoadMap>("change_map");
  nav_msgs::LoadMap srv;
  srv.request.map_url = "/home/developer/my_ros_ws/src/AR100/anscer_navigation/maps/map_part1.yaml";
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service ");
    return 1;
  }

  return 0;
}

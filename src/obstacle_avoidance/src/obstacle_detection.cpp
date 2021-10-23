// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/ObstacleDetected.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "obstacle_detection");
  ros::NodeHandle nh;

  //ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
  //ros::spin();
  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}

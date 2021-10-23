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

  ros::Publisher pub = nh.advertise<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 1);
  tm_msgs::ObstacleDetected msg;
  msg.obstacle_detected = true;
  ROS_INFO_STREAM("invio msg");
  pub.publish(msg);
  ros::spin();




  return 0;

}

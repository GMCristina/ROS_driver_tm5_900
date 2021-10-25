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
  while (pub.getNumSubscribers() < 1) {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
    ROS_INFO_STREAM("wait for publisher");
  }
  // ros::Duration(10).sleep();
  tm_msgs::ObstacleDetected msg;
  msg.obstacle_detected = true;
  pub.publish(msg);
  ROS_INFO_STREAM("Inviato messaggio obstacle_detected");
  ros::spin();




  return 0;

}

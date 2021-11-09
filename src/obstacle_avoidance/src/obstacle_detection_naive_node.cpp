// Node to simulate an obstacle detection
// Run this node to send a message of ObstacleDetected

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <cstdlib>

// Custom message ObstacleDetected
#include "tm_msgs/ObstacleDetected.h"

int main(int argc, char **argv)
{
  //Node initialization
  ros::init(argc, argv, "obstacle_detection_naive_node");
  ros::NodeHandle nh;

  // Create publisher for ObstacleDetected message
  ros::Publisher pub = nh.advertise<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 1);

  // IMPORTANT: Wait for the publisher to be ready (otherwise message is lost)
  while (pub.getNumSubscribers() < 1) {
    ros::WallDuration sleep_t(0.5); //seconds
    sleep_t.sleep();
    ROS_INFO_STREAM("OB DETECTED: Wait for publisher");
  }

  //Create ObstacleDetected message
  tm_msgs::ObstacleDetected msg;
  msg.obstacle_detected = true;

  //Publish ObstacleDetected message
  pub.publish(msg);
  ROS_INFO_STREAM("OB DETECTED: Sent message obstacle_detected");

  //IMPORTANT: Wait to give the node time to send the message (otherwise message is lost)
  ros::Duration(1).sleep(); //seconds

  return 0;
}

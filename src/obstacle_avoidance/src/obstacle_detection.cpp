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
  while (pub.getNumSubscribers() < 1) { //serve per aspettare che si crei la connessione (senza no msg)
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
    ROS_INFO_STREAM("OB DETECTED: wait for publisher");
  }

  tm_msgs::ObstacleDetected msg;
  msg.obstacle_detected = true;
  pub.publish(msg);
  ROS_INFO_STREAM("OB DETECTED: Inviato messaggio obstacle_detected");
  //ros::spin();
  ros::Duration(1).sleep(); //serve per dargli tempo di inviare il msg (senza no msg)



  return 0;

}

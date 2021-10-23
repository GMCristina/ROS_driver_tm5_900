// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SendScript.h"

int main(int argc, char **argv)
{
  //std::string cmd = "PTP(\"JPP\",0,0,90,0,90,0,35,200,0,false)";
  std::string cmd = "StopAndClearBuffer()";

  ros::init(argc, argv, "obstacle_avoidance_naive");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/obstacle_avoidance_naive");
  tm_msgs::SendScript srv;

  //Request
  srv.request.id = "obs";
  srv.request.script = cmd;

  if (client.call(srv))
  {
    if (srv.response.ok) ROS_INFO_STREAM("Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error send script to robot");
    ros::Duration(10).sleep();
    return 1;
  }

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}

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
  std::string cmd2 = "StopAndClearBuffer()\r\nResume()\r\nPTP(\"JPP\",0,0,0,0,0,0,35,200,0,false)"; //home
  std::string cmd3 = "Move_PTP(\"JPP\",90,0,0,0,0,0,5,200,0,false)"; // J1 (spalla1)
  std::string cmd4 = "Move_PTP(\"JPP\",0,0,0,0,0,180,5,200,0,false)"; // J6 (polso3)

  ros::init(argc, argv, "obstacle_avoidance_naive");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
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
    //ros::Duration(10).sleep();
    return 1;
  }

  //ROS_INFO_STREAM_NAMED("demo_sendscript", "shutdown.");
  return 0;

}

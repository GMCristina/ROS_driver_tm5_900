// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SendScript.h"
#include "tm_msgs/ObstacleDetected.h"

void KillCallback(bool* flag,ros::NodeHandle &nh, const tm_msgs::ObstacleDetected::ConstPtr& msg){
  ROS_INFO_STREAM("loop_trajectory kill");
  flag = false;
  ros::shutdown();
}

int main(int argc, char **argv)
{

  bool flag_loop = true;

  ros::init(argc, argv, "loop_trajectory");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;

  ros::Subscriber sub = nh.subscribe<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 5, boost::bind(&flag_loop, &KillCallback, boost::ref(nh), _1));

  while(flag_loop) {
    std::string cmd1 = "Move_PTP(\"JPP\",90,0,0,0,0,0,5,200,0,false)\r\nMove_PTP(\"JPP\",-90,0,0,0,0,0,5,200,0,false)";
    srv.request.id = "tr1";
    srv.request.script = cmd1;

    if (client.call(srv))
    {
      if (srv.response.ok) ROS_INFO_STREAM("Sent script to robot");
      else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
    }
    else
    {
      ROS_ERROR_STREAM("Error send script to robot");
      return 1;
    }
    ros::Duration(20).sleep();
/*
    ros::Duration(2).sleep();

    std::string cmd2 = "Move_PTP(\"JPP\",-90,0,0,0,0,0,5,200,0,false)";
    srv.request.id = "tr2";
    srv.request.script = cmd2;

    if (client.call(srv))
    {
      if (srv.response.ok) ROS_INFO_STREAM("Sent script to robot");
      else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
    }
    else
    {
      ROS_ERROR_STREAM("Error send script to robot");
      return 1;
    }
    ros::Duration(2).sleep();
*/
    ros::spinOnce();

  }
    ROS_INFO_STREAM("uscito dal loop")
    return 0;

}

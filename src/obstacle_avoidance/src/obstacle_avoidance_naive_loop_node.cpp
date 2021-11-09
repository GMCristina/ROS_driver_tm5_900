// Node for obstacle avoidance with loop trajectory
// when ObstacleDetected message is received
// the trajectory stops and the robot returns to home state

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <cstdlib>

#include "tm_msgs/SendScript.h"

// Custom message ObstacleDetected
#include "tm_msgs/ObstacleDetected.h"

#include "ros/this_node.h"
#include "ros/master.h"
#include <boost/bind.hpp>

// Callback executed when ObstacleDetected message is received
void ObstacleDetectedCallback(ros::NodeHandle &nh, const tm_msgs::ObstacleDetected::ConstPtr& msg)
  {
    std::string flag = msg->obstacle_detected ? "true" : "false";
    ROS_INFO_STREAM("LOOP: Received message ObstacleDetected: " << flag );

    // First command needed to stop immediately the trajectory
    ROS_INFO_STREAM("LOOP: First clear");

    // Create the client to request the service SendScript
    ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");

    // Create the request with the commands to stop the trajectory
    std::string cmd2 = "StopAndClearBuffer()";
    tm_msgs::SendScript srv;
    srv.request.id = "obs1";
    srv.request.script = cmd2;

    // Service call
    if (client.call(srv))
      {
        if (srv.response.ok) ROS_INFO_STREAM("LOOP: Sent script to robot");
        else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
      }
    else
      {
        ROS_ERROR_STREAM("LOOP: Error send script to robot");
      }

    //IMPORTANT: Wait for loop_trajectory_node to stop
    ROS_INFO_STREAM("LOOP: Wait for loop_trajectory_node to stop");
    bool waiting = true;
    std::string node = "/loop_trajectory_node";
    while(waiting) {
      ros::V_string v;
      ros::master::getNodes(v);
      waiting = false;
      for (int i=0; i < v.size(); i++) {
            if (v[i].compare(node)==0) {
              ROS_INFO_STREAM("LOOP: Still exists " << v[i] <<"node... wait..." );
              waiting = true;
            }
      }
      ros::Duration(0.5).sleep();
    }
    ROS_INFO_STREAM("LOOP: loop_trajectory_node terminated, send clear and home...");

    // Create the client to request the service SendScript
    ros::ServiceClient client1 = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    // Create the request with the commands to stop the trajectory and return to home state
    std::string cmd = "StopAndClearBuffer()\r\nPTP(\"JPP\",0,0,0,0,0,0,10,200,0,false)"; //stop+home
    tm_msgs::SendScript srv1;
    srv1.request.id = "obs2";
    srv1.request.script = cmd;

    // Service call
    if (client1.call(srv1))
    {
      if (srv1.response.ok) ROS_INFO_STREAM("LOOP: Sent script to robot");
      else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
    }else{
        ROS_ERROR_STREAM("LOOP: Error send script to robot");
        }
  }

int main(int argc, char **argv)
{
  //Node initialization
  ros::init(argc, argv, "obstacle_avoidance_naive_loop_node");
  ros::NodeHandle nh;

  // Create subscriber to receive ObstacleDetected messages
  ros::Subscriber sub = nh.subscribe<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 5, boost::bind(&ObstacleDetectedCallback, boost::ref(nh), _1));
  // Keep the node active to receive messages
  ros::spin();

  return 0;
}

// Node for obstacle avoidance with trajectory planned in Moveit
// when ObstacleDetected message is received
// the trajectory stops and the robot returns to home state

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <cstdlib>

#include "tm_msgs/SendScript.h"

// Custom message ObstacleDetected
#include "tm_msgs/ObstacleDetected.h"

#include <boost/bind.hpp>

// Callback executed when ObstacleDetected message is received
void ObstacleDetectedCallback(ros::NodeHandle &nh, const tm_msgs::ObstacleDetected::ConstPtr& msg)
  {
    std::string flag = msg->obstacle_detected ? "true" : "false";
    ROS_INFO_STREAM("MOVEIT: Received message ObstacleDetected: " << flag );

    ROS_INFO_STREAM("MOVEIT: Send stop + home");

    // Create the client to request the service SendScript
    ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");

    // Create the request with the desired commands to handle obstacle detection
    std::string cmd = "StopAndClearBuffer()\r\nPTP(\"JPP\",0,0,0,0,0,0,10,200,0,false)"; //stop+home
    std::string cmd1 = "StopAndClearBuffer()"; //stop

    tm_msgs::SendScript srv;

    srv.request.id = "Obs";
    srv.request.script = cmd;

    // Service call
      if (client.call(srv))
      {
        if (srv.response.ok) ROS_INFO_STREAM("MOVEIT: Sent script to robot");
        else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
      }
      else
      {
        ROS_ERROR_STREAM("MOVEIT: Error send script to robot");
      }
  }

int main(int argc, char **argv)
{
  //Node initialization
  ros::init(argc, argv, "obstacle_avoidance_naive_moveit_node");
  ros::NodeHandle nh;

  // Create subscriber to receive ObstacleDetected messages
  ros::Subscriber sub = nh.subscribe<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 5, boost::bind(&ObstacleDetectedCallback, boost::ref(nh), _1));
  // Keep the node active to receive messages
  ros::spin();

  return 0;
}

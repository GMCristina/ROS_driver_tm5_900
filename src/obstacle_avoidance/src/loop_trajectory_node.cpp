//Node for a simple loop trajectory with external script
// J1 moves between 0° and 90°

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <cstdlib>

#include "tm_msgs/SendScript.h"
#include "tm_msgs/StaResponse.h"

// Custom message ObstacleDetected
#include "tm_msgs/ObstacleDetected.h"

// Callback executed when ObstacleDetected message is received to kill the node
// (otherwise it keeps on sending trajectory commands)
void KillCallback(bool* flag_loop_p,ros::NodeHandle &nh, const tm_msgs::ObstacleDetected::ConstPtr& msg){
  std::string flag = msg->obstacle_detected ? "true" : "false";
  ROS_INFO_STREAM("LOOP: Received message ObstacleDetected: " << flag );

  // Kill the node by setting the flag_loop variable to false
  ROS_INFO_STREAM("LOOP: loop_trajectory_node killing...");
  *flag_loop_p = false;
  //ros::shutdown();
}

// Callback executed when a TM_STA message is received to implement a loop trajectory
void StaResponseCallback(bool* flag_done_p, const tm_msgs::StaResponse::ConstPtr& msg)
{
  //Check if it is the TMSTA message in response to the QueueTag of the trajectory commands
  ROS_INFO_STREAM("TMSTA message: subcmd is = " << msg->subcmd << ", subdata is " << msg->subdata);
  std::string comando = "01"; //QueueTag
  std::string data = "01,true"; // Tag number = 01, Status = true

  if((msg->subcmd.compare(comando)==0)&&(msg->subdata.compare(data)==0)) {
    ROS_INFO_STREAM("LOOP: Trajectory executed!");
    *flag_done_p = true;
  }
}

int main(int argc, char **argv)
{
  //Flag initialization
  bool flag_loop = true; // loop trajectory on
  bool flag_done = false; // trajectory commands still not executed

  //Node initialization
  ros::init(argc, argv, "loop_trajectory_node");
  ros::NodeHandle nh;

  //IMPORTANT: Wait for TM_SCT connection (otherwise the node tries to send script before the connection is established)
  ROS_INFO_STREAM("LOOP: Wait for TM_SCT connection...");
  ros::Duration(1).sleep(); //seconds

  // Create subscriber to receive ObstacleDetected messages
  ros::Subscriber sub = nh.subscribe<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 5, boost::bind(&KillCallback,&flag_loop, boost::ref(nh), _1));
  // Create subscriber to receive StaResponse messages
  ros::Subscriber sub2 = nh.subscribe<tm_msgs::StaResponse>("tm_driver/sta_response", 1000, boost::bind(&StaResponseCallback,&flag_done,_1));

  // Create the client to request the service SendScript
  ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");

  // Create the request with the first commands for the loop trajectory
  tm_msgs::SendScript srv;
  std::string cmd = "Move_PTP(\"JPP\",90,0,0,0,0,0,5,200,0,false)\r\nMove_PTP(\"JPP\",-90,0,0,0,0,0,5,200,0,false)\r\nQueueTag(1)";
  srv.request.id = "loop";
  srv.request.script = cmd;

  ROS_INFO_STREAM("LOOP: First command...");

  // Service call
  if (client.call(srv))
  {
    if (srv.response.ok) ROS_INFO_STREAM("LOOP: Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("LOOP: Error send script to robot");
    return 1;
  }

  // Trajectory loop
  while(flag_loop) {
      if(flag_done){
            //Send new trajectory command
            ROS_INFO_STREAM("LOOP: New trajectory command");
            //Check for messages
            ros::spinOnce();
            if (client.call(srv))
            {
              if (srv.response.ok) ROS_INFO_STREAM("LOOP: Sent script to robot");
              else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
            }
            else
            {
              ROS_ERROR_STREAM("LOOP: Error send script to robot");
              return 1;
            }
            flag_done = false;
      } else {
        // Wait TM_STA message of trajectory executed
        ROS_INFO_STREAM("LOOP: Wait trajectory executed");
        //Check for messages
        ros::spinOnce();
        ros::Duration(1).sleep();
      }
    //Check for messages
    ros::spinOnce();

  }
    // Node shutdown for obstacle detection
    ROS_INFO_STREAM("LOOP: loop trajectory killed");
    return 0;
}

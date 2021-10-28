// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SendScript.h"
#include "tm_msgs/ObstacleDetected.h"
#include "tm_msgs/StaResponse.h"

void KillCallback(bool* flag_loop_p,ros::NodeHandle &nh, const tm_msgs::ObstacleDetected::ConstPtr& msg){
  std::string flag = msg->obstacle_detected ? "true" : "false";
  ROS_INFO_STREAM("LOOP: Ricevuto messaggio ObstacleDetected: " << flag );
  ROS_INFO_STREAM("LOOP: Loop_trajectory killing...");
  *flag_loop_p = false;
  //ros::shutdown();
}

void StaResponseCallback(bool* flag_done_p, const tm_msgs::StaResponse::ConstPtr& msg)
{
  ROS_INFO_STREAM("StaResponse: subcmd is = " << msg->subcmd << ", subdata is " << msg->subdata);
  std::string comando = "01";
  std::string data = "01,true";

  if((msg->subcmd.compare(comando)==0)&&(msg->subdata.compare(data)==0)) {
    ROS_INFO_STREAM("LOOP: Traiettoria eseguita!");
    *flag_done_p = true;
  }
}

int main(int argc, char **argv)
{


  bool flag_loop = true;
  bool flag_done = false;

  ros::init(argc, argv, "loop_trajectory");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("LOOP: attesa connessione...");
  ros::Duration(1).sleep();
  ROS_INFO_STREAM("LOOP: inizio..");

  ros::Subscriber sub = nh.subscribe<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 5, boost::bind(&KillCallback,&flag_loop, boost::ref(nh), _1));
  ros::Subscriber sub2 = nh.subscribe<tm_msgs::StaResponse>("tm_driver/sta_response", 1000, boost::bind(&StaResponseCallback,&flag_done,_1));

  ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;

  std::string cmd = "Move_PTP(\"JPP\",90,0,0,0,0,0,5,200,0,false)\r\nMove_PTP(\"JPP\",-90,0,0,0,0,0,5,200,0,false)";
  std::string cmd2 = "Move_PTP(\"JPP\",90,0,0,0,0,0,5,200,0,false)\r\nMove_PTP(\"JPP\",-90,0,0,0,0,0,5,200,0,false)\r\nQueueTag(1)";
  srv.request.id = "trajec";
  srv.request.script = cmd2;
  ROS_INFO_STREAM("LOOP: Invio primo comando...");
  if (client.call(srv))
  {
    if (srv.response.ok) ROS_INFO_STREAM("LOOP: Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("LOOP: Error send script to robot");
  //  return 1;
  }

  while(flag_loop) {
      if(flag_done){
            ROS_INFO_STREAM("LOOP: Nuovo comando traiettoria");
            ros::spinOnce();
            if (client.call(srv))
            {
              if (srv.response.ok) ROS_INFO_STREAM("LOOP: Sent script to robot");
              else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
            }
            else
            {
              ROS_ERROR_STREAM("LOOP: Error send script to robot");
            //  return 1;
            }
            flag_done = false;
      } else {
        ROS_INFO_STREAM("LOOP: Attesa comando eseguito");
        ros::spinOnce();
        ros::Duration(1).sleep();
      }
    // ros::Duration(20).sleep();
    ros::spinOnce(); //controllare!

  }
    ROS_INFO_STREAM("LOOP: loop trajectory killed");
    return 0;

}

// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SendScript.h"
#include "tm_msgs/ObstacleDetected.h"

#include "ros/this_node.h"
#include "ros/master.h"

#include <boost/bind.hpp>

void ObstacleDetectedCallback(ros::NodeHandle &nh, const tm_msgs::ObstacleDetected::ConstPtr& msg)
  {
    std::string flag = msg->obstacle_detected ? "true" : "false";
    ROS_INFO_STREAM("NAIVE3: Ricevuto messaggio ObstacleDetected: " << flag );
    ROS_INFO_STREAM("NAIVE3: Primo clear");
      //std::string cmd = "PTP(\"JPP\",0,0,90,0,90,0,35,200,0,false)";
      std::string cmd2 = "StopAndClearBuffer()";
      ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
      tm_msgs::SendScript srv;

      srv.request.id = "pause";
      srv.request.script = cmd2;

      if (client.call(srv))
      {
        if (srv.response.ok) ROS_INFO_STREAM("NAIVE3: Sent script to robot");
        else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
      }
      else
      {
        ROS_ERROR_STREAM("NAIVE3: Error send script to robot");
      }


    ROS_INFO_STREAM("NAIVE3: Attesa termine loop");
    bool attesa = true;
    std::string nodo = "/loop_trajectory";
    while(attesa) {
      ros::V_string v;
      ros::master::getNodes(v);
      attesa = false;
      //ROS_INFO_STREAM("Elenco nodi: " << "( "<<v.size()<<")");
      for (int i=0; i < v.size(); i++) {
            //ROS_INFO_STREAM("Nodo: " << v[i] );
            if (v[i].compare(nodo)==0) {
              ROS_INFO_STREAM("NAIVE3: Ancora esiste nodo " << v[i] <<"... aspetto..." );
              attesa = true;
            }
      }
      ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("NAIVE3: Loop terminato, send Clear and home...");

    std::string cmd = "StopAndClearBuffer()\r\nPTP(\"JPP\",0,0,0,0,0,0,10,200,0,false)"; //home
    ros::ServiceClient client1 = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    tm_msgs::SendScript srv1;

    srv1.request.id = "pause";
    srv1.request.script = cmd;

    if (client1.call(srv))
    {
      if (srv1.response.ok) ROS_INFO_STREAM("NAIVE3: Sent script to robot");
      else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
    }else{
        ROS_ERROR_STREAM("NAIVE3: Error send script to robot");
        }
  }

int main(int argc, char **argv)
{

  ros::init(argc, argv, "obstacle_avoidance_naive2");
  ros::NodeHandle nh;
//  ros::Subscriber sub = nh.subscribe("tm_driver/obstacle_detected", 5, boost::bind(ObstacleDetectedCallback, &nh, _1));
  ros::Subscriber sub = nh.subscribe<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 5, boost::bind(&ObstacleDetectedCallback, boost::ref(nh), _1));
  ros::spin();

  return 0;

}

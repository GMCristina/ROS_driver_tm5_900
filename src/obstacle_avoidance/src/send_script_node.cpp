//Node to test external script functions

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <cstdlib>

#include "tm_msgs/SendScript.h"

int main(int argc, char **argv)
{
  // Commands tested
  std::string cmd = "StopAndClearBuffer()"; //stop
  std::string cmd2 = "StopAndClearBuffer()\r\nResume()\r\nPTP(\"JPP\",0,0,0,0,0,0,35,200,0,false)"; //stop+home
  std::string cmd3 = "Move_PTP(\"JPP\",90,0,0,0,0,0,5,200,0,false)"; // J1
  std::string cmd4 = "Move_PTP(\"JPP\",0,0,0,0,0,180,5,200,0,false)"; // J6
  std::string cmd5 = "PVTEnter(1)\r\nPVTPoint(517.5,-236.6,574.10,90,90,0,0,0,0,0,0,0,1)\r\nQueueTag(1)\r\nPVTExit()"; //cartesian
  std::string cmd6 = "PTP(\"CPP\",517.5,-236.6,574.10,90,90,0,10,200,0,false)";
  std::string cmd7 = "PVTEnter(0)\r\nPVTPoint(90,0,0,0,0,0,0,0,0,0,0,0,0.125)\r\nQueueTag(1)\r\nPVTExit()"; //joint

  std::stringstream ss;
  ss << "PVTEnter(0)\r\n";
  ss << "PVTPoint(0.00055,-0.00058,11.24994,0.00056,-11.24932,-0.00010,0.00115,-0.00191,30.64505,0.00105,-30.64301,0.00023,0.316165)\r\n";
  ss << "PVTPoint(0.00097,-0.00128,22.50001,0.00094,-22.49864,-0.00002,0.00187,-0.00309,49.64146,0.00169,-49.63817,0.00037,0.129315)\r\n";
  ss << "PVTPoint(0.00140,-0.00198,33.75007,0.00133,-33.74796,0.00007,0.00230,-0.00379,60.92482,0.00208,-60.92078,0.00046,0.100835)\r\n";
  ss << "PVTPoint(0.00182,-0.00268,45.00014,0.00171,-44.99728,0.00015,0.00246,-0.00407,65.41458,0.00223,-65.41024,0.00049,0.085145)\r\n";
  ss << "PVTPoint(0.00225,-0.00338,56.25021,0.00210,-56.24660,0.00024,0.00226,-0.00373,59.99778,0.00205,-59.99380,0.00045,0.086855)\r\n";
  ss << "PVTPoint(0.00267,-0.00408,67.50027,0.00248,-67.49591,0.00032,0.00185,-0.00306,49.14996,0.00168,-49.14670,0.00037,0.101845)\r\n";
  ss << "PVTPoint(0.00309,-0.00478,78.75034,0.00286,-78.74523,0.00041,0.00115,-0.00190,30.49825,0.00104,-30.49623,0.00023,0.13061)\r\n";
  ss << "PVTPoint(0.00352,-0.00548,90.00040,0.00325,-89.99455,0.00049,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.31375)\r\n";
  ss << "QueueTag(1)\r\nPVTExit()";
  std::string cmd8 = ss.str();
  //ROS_INFO_STREAM("Comando: "<<cmd8);

  std::string cmd9 = "Move_PTP(\"JPP\",-270,0,0,0,0,0,5,1000,0,false)";

  //Node initialization
  ros::init(argc, argv, "send_script_node");
  ros::NodeHandle nh;

  //IMPORTANT: Wait for TM_SCT connection (otherwise the node tries to send script before the connection is established)
  ROS_INFO_STREAM("NAIVE: Wait for TM_SCT connection...");
  ros::Duration(1).sleep();

  // Create the client to request the service SendScript
  ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  // Create the request with the desired commands to test
  tm_msgs::SendScript srv;
  srv.request.id = "obs";
  srv.request.script = cmd9;

  // Service call
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

  return 0;
}

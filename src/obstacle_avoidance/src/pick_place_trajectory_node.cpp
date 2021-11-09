//Node for pick and place trajectory with external script

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <cstdlib>

// Custom message ObstacleDetected
#include "tm_msgs/ObstacleDetected.h"

#include "tm_msgs/SendScript.h"
#include "tm_msgs/StaResponse.h"

// Callback executed when ObstacleDetected message is received to kill the node
// (otherwise it keeps on sending trajectory commands)
void KillCallback(bool* flag_loop_p,ros::NodeHandle &nh, const tm_msgs::ObstacleDetected::ConstPtr& msg){
  std::string flag = msg->obstacle_detected ? "true" : "false";
  ROS_INFO_STREAM("PICK: Received message ObstacleDetected: " << flag );

  // Kill the node by setting the flag_loop variable to false
  ROS_INFO_STREAM("PICK: pick_place_trajectory_node killing...");
  *flag_loop_p = false;
  //ros::shutdown();
}

// Callback executed when a TM_STA message is received to implement a loop trajectory
void StaResponseCallback(bool* flag_done_p, const tm_msgs::StaResponse::ConstPtr& msg)
{
  //Check if it is the TMSTA message in response to the QueueTag of the trajectory commands
  ROS_INFO_STREAM("TMSTA message: subcmd is = " << msg->subcmd << ", subdata is " << msg->subdata);
  std::string cmd = "01"; //QueueTag
  std::string data = "01,true"; // Tag number = 01, Status = true

  if((msg->subcmd.compare(cmd)==0)&&(msg->subdata.compare(data)==0)) {
    ROS_INFO_STREAM("PICK: Trajectory executed!");
    *flag_done_p = true;
  }
}

int main(int argc, char **argv)
{
  //Flag initialization
  bool flag_loop = true; // loop trajectory on
  bool flag_done = false; // trajectory commands still not executed

  //Node initialization
  ros::init(argc, argv, "pick_place_trajectory_node");
  ros::NodeHandle nh;

  //IMPORTANT: Wait for TM_SCT connection (otherwise the node tries to send script before the connection is established)
  ROS_INFO_STREAM("PICK: Wait for TM_SCT connection...");
  ros::Duration(1).sleep(); //seconds

  // Create subscriber to receive ObstacleDetected messages
  ros::Subscriber sub = nh.subscribe<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 5, boost::bind(&KillCallback,&flag_loop, boost::ref(nh), _1));
  // Create subscriber to receive StaResponse messages
  ros::Subscriber sub2 = nh.subscribe<tm_msgs::StaResponse>("tm_driver/sta_response", 1000, boost::bind(&StaResponseCallback,&flag_done,_1));

  // Create the client to request the service SendScript
  ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  // Create the request with the commands for the pick and place trajectory
  tm_msgs::SendScript srv;

  // 1: Pick and place trajectory with PTP
  std::stringstream ss;
  ss << "PTP(\"JPP\",-10.88,23.75,86.66,-18.56,92.41,0,5,200,0,false)\r\n"; //1 pick
  ss << "PTP(\"JPP\",-10.88,3.23,86.66,-4.20,92.41,0,5,200,0,false)\r\n"; //2
  ss << "PTP(\"JPP\",28.38,3.23,86.66,-4.20,92.41,0,5,200,0,false)\r\n"; //3
  ss << "PTP(\"JPP\",28.38,23.75,86.66,-18.56,92.41,0,5,200,0,false)\r\n"; //4 place
  ss << "PTP(\"JPP\",28.38,3.23,86.66,-4.20,92.41,0,5,200,0,false)\r\n"; //3
  ss << "PTP(\"JPP\",-10.88,3.23,86.66,-4.20,92.41,0,5,200,0,false)\r\n"; //2
  ss << "QueueTag(1)";
  std::string cmd2 = ss.str();
  //ROS_INFO_STREAM("PICK Cmd: " << cmd2);

  // 2: Pick and place trajectory with PVTPoint (from Moveit) and Move_PTP
  std::stringstream ss3;
  ss3 << "PVTEnter(0)\r\n";
  ss3 << "PVTPoint(-1.08811,2.37447,8.66625,-1.85596,9.24145,-0.00043,-19.42402,42.37978,154.68622,-33.13028,164.95078,-0.00436,0.09520)\r\n";
  ss3 << "PVTPoint(-2.17636,4.74882,17.33262,-3.71210,18.48290,-0.00067,-31.63423,69.02029,251.92411,-53.95644,268.64111,-0.00710,0.03969)\r\n";
  ss3 << "PVTPoint(-3.26460,7.12316,25.99900,-5.56825,27.72435,-0.00092,-39.25025,85.63710,312.57549,-66.94659,333.31715,-0.00881,0.03035)\r\n";
  ss3 << "PVTPoint(-4.35284,9.49751,34.66537,-7.42439,36.96580,-0.00116,-45.10179,98.40412,359.17509,-76.92717,383.00898,-0.01012,0.02552)\r\n";
  ss3 << "PVTPoint(-5.44108,11.87186,43.33174,-9.28053,46.20725,-0.00141,-46.88508,102.29494,373.37658,-79.96881,398.15284,-0.01052,0.02288)\r\n";
  ss3 << "PVTPoint(-6.52932,14.24621,51.99812,-11.13667,55.44870,-0.00165,-43.49639,94.90143,346.39028,-74.18895,369.37581,-0.00976,0.02355)\r\n";
  ss3 << "PVTPoint(-7.61756,16.62056,60.66449,-12.99281,64.69015,-0.00189,-37.26279,81.30081,296.74802,-63.55670,316.43941,-0.00836,0.02669)\r\n";
  ss3 << "PVTPoint(-8.70580,18.99491,69.33086,-14.84895,73.93160,-0.00214,-30.57855,66.71699,243.51705,-52.15583,259.67618,-0.00686,0.03225)\r\n";
  ss3 << "PVTPoint(-9.79404,21.36925,77.99723,-16.70509,83.17305,-0.00238,-19.41983,42.37064,154.65286,-33.12314,164.91521,-0.00436,0.03970)\r\n";
  ss3 << "PVTPoint(-10.88228,23.74360,86.66361,-18.56123,92.41450,-0.00263,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.09520)\r\n"; //1 pick
  ss3 << "PVTExit()\r\n";
  ss3 << "Move_PTP(\"JPP\",0,-20.52,0,14.36,0,0,10,200,0,false)\r\n"; // 2
  ss3 << "Move_PTP(\"JPP\",39.26,0,0,0,0,0,10,200,0,false)\r\n"; // 3
  ss3 << "Move_PTP(\"JPP\",0,20.52,0,-14.36,0,0,10,200,0,false)\r\n"; // 4 place
  ss3 << "Move_PTP(\"JPP\",0,-20.52,0,14.36,0,0,10,200,0,false)\r\n"; // 3
  ss3 << "Move_PTP(\"JPP\",-39.26,0,0,0,0,0,10,200,0,false)\r\n"; // 2
  ss3 << "QueueTag(1)";
  std::string cmd3 = ss3.str();
  //ROS_INFO_STREAM("PICK: Cmd: " << cmd3);

  srv.request.id = "pick1";
  srv.request.script = cmd3;
  ROS_INFO_STREAM("PICK: First command...");

  // Service call
  if (client.call(srv))
  {
    if (srv.response.ok) ROS_INFO_STREAM("PICK: Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("PICK: Error send script to robot");
    return 1;
  }

  // Create the request with the commands for the loop pick and place trajectory
  tm_msgs::SendScript srv1;
  std::stringstream ss4;
  ss4 << "Move_PTP(\"JPP\",0,20.52,0,-14.36,0,0,10,200,0,false)\r\n"; // 2->1
  ss4 << "Move_PTP(\"JPP\",0,-20.52,0,14.36,0,0,10,200,0,false)\r\n"; // 2
  ss4 << "Move_PTP(\"JPP\",39.26,0,0,0,0,0,10,200,0,false)\r\n"; // 3
  ss4 << "Move_PTP(\"JPP\",0,20.52,0,-14.36,0,0,10,200,0,false)\r\n"; // 4 place
  ss4 << "Move_PTP(\"JPP\",0,-20.52,0,14.36,0,0,10,200,0,false)\r\n"; // 3
  ss4 << "Move_PTP(\"JPP\",-39.26,0,0,0,0,0,10,200,0,false)\r\n"; // 2
  ss4 << "QueueTag(1)";
  std::string cmd4 = ss4.str();
  //ROS_INFO_STREAM("PICK: Loop commands " << cmd4);
  srv1.request.id = "pick";
  srv1.request.script = cmd4;

  // Pick and place trajectory loop
  while(flag_loop) {
      if(flag_done){
            //Send new trajectory command
            ROS_INFO_STREAM("PICK: New trajectory command");
            // Check for messages
            ros::spinOnce();
            if (client.call(srv1))
            {
              if (srv.response.ok) ROS_INFO_STREAM("PICK: Sent script to robot");
              else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
            }
            else
            {
              ROS_ERROR_STREAM("PICK: Error send script to robot");
              return 1;
            }
            flag_done = false;
      } else {
        // Wait TM_STA message of trajectory executed
        ROS_INFO_STREAM("PICK: Wait trajectory executed");
        // Check for messages
        ros::spinOnce();
        ros::Duration(0.5).sleep();
      }
    // Check for messages
    ros::spinOnce();
  }
    // Node shutdown for obstacle detection
    ROS_INFO_STREAM("PICK: loop trajectory killed");
    return 0;
}

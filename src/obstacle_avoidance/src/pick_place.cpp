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
  ROS_INFO_STREAM("PICK: Ricevuto messaggio ObstacleDetected: " << flag );
  ROS_INFO_STREAM("PICK: Loop_trajectory killing...");
  *flag_loop_p = false;
  //ros::shutdown();
}

void StaResponseCallback(bool* flag_done_p, const tm_msgs::StaResponse::ConstPtr& msg)
{
  ROS_INFO_STREAM("StaResponse: subcmd is = " << msg->subcmd << ", subdata is " << msg->subdata);
  std::string comando = "01";
  std::string data = "01,true";

  if((msg->subcmd.compare(comando)==0)&&(msg->subdata.compare(data)==0)) {
    ROS_INFO_STREAM("PICK: Traiettoria eseguita!");
    *flag_done_p = true;
  }
}

int main(int argc, char **argv)
{

  bool flag_loop = true;
  bool flag_done = false;

  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("PICK: attesa connessione...");
  ros::Duration(1).sleep();
  ROS_INFO_STREAM("PICK: inizio..");

  ros::Subscriber sub = nh.subscribe<tm_msgs::ObstacleDetected>("tm_driver/obstacle_detected", 5, boost::bind(&KillCallback,&flag_loop, boost::ref(nh), _1));
  ros::Subscriber sub2 = nh.subscribe<tm_msgs::StaResponse>("tm_driver/sta_response", 1000, boost::bind(&StaResponseCallback,&flag_done,_1));

  ros::ServiceClient client = nh.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;


  //std::string cmd2 = "Move_PTP(\"JPP\",90,0,0,0,0,0,5,200,0,false)\r\nMove_PTP(\"JPP\",-90,0,0,0,0,0,5,200,0,false)\r\nQueueTag(1)";
  std::stringstream ss;
  ss << "PTP(\"JPP\",-10.88,23.75,86.66,-18.56,92.41,0,5,200,0,false)\r\n"; //1 pick
  ss << "PTP(\"JPP\",-10.88,3.23,86.66,-4.20,92.41,0,5,200,0,false)\r\n"; //2
  ss << "PTP(\"JPP\",28.38,3.23,86.66,-4.20,92.41,0,5,200,0,false)\r\n"; //3
  ss << "PTP(\"JPP\",28.38,23.75,86.66,-18.56,92.41,0,5,200,0,false)\r\n"; //4 place
  ss << "PTP(\"JPP\",28.38,3.23,86.66,-4.20,92.41,0,5,200,0,false)\r\n"; //3
  ss << "PTP(\"JPP\",-10.88,3.23,86.66,-4.20,92.41,0,5,200,0,false)\r\n"; //2
  //ss << "PTP(\"JPP\",0,0,0,0,0,0,5,200,0,false)\r\n";
  ss << "QueueTag(1)";
  std::string cmd2 = ss.str();
  ROS_INFO_STREAM("PICK Comando: " << cmd2);

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
  ROS_INFO_STREAM("PICK Comando: " << cmd3);



  srv.request.id = "trajec";
  srv.request.script = cmd3;
  ROS_INFO_STREAM("PICK: Invio primo comando...");

  if (client.call(srv))
  {
    if (srv.response.ok) ROS_INFO_STREAM("PICK: Sent script to robot");
    else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("PICK: Error send script to robot");
  //  return 1;
  }


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
  ROS_INFO_STREAM("PICK Comando: " << cmd4);

  srv1.request.id = "loop";
  srv1.request.script = cmd4;




  while(flag_loop) {
      if(flag_done){
            ROS_INFO_STREAM("PICK: Nuovo comando traiettoria");
            ros::spinOnce();
            if (client.call(srv1))
            {
              if (srv.response.ok) ROS_INFO_STREAM("PICK: Sent script to robot");
              else ROS_WARN_STREAM("Sent script to robot , but response not yet ok ");
            }
            else
            {
              ROS_ERROR_STREAM("PICK: Error send script to robot");
            //  return 1;
            }
            flag_done = false;
      } else {
        ROS_INFO_STREAM("PICK: Attesa comando eseguito");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
      }
    // ros::Duration(20).sleep();
    ros::spinOnce(); //controllare!

  }
    ROS_INFO_STREAM("PICK: loop trajectory killed");
    return 0;

}

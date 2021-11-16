# __User Guide for TM ROS Driver__

This repo is forked from [tmr_ros1]( https://github.com/TechmanRobotInc/tmr_ros1) that provides ROS support for techman robots. The original repo has been further developed using the existing driver to simulate and handle an ¬__external obstacle detection__. <br/>
Specifically the repo has been tested for cobot __TM5-900__ with __ROS Melodic__ under __Ubuntu 18.04__.<br/>

For further information about the existing ROS driver refers to [tmr_ros1]( https://github.com/TechmanRobotInc/tmr_ros1); for details about the project refers to _documents/Report.pdf_.<br/>

## __1.ROS Driver__

The existing TM ROS driver connects to _TMflow Ethernet Slave_ and to a _Listen node_ (running at a _TMflow project_). This allows the driver to get the robot state and to control the robot using _External Script_. More information about _TM Robot Expression_ and _Ethernet Slave_, see [Expression Editor and Listen Node.pdf](https://assets.omron.eu/downloads/manual/en/v1/i848_tm_expression_editor_and_listen_node_reference_manual_en.pdf). <br/>
The TM ROS driver for ROS1 is a __single ROS node__ that handles robot-pc communication offering the following interfaces:

> __Action Server__
>
> - An  action interface on _/follow_joint_trajectory_ for integration with MoveIt; the action allows the execution of the trajectory planned on Moveit
>
> __Topic Publisher__
>
> - publish feedback state on _/feedback_states_  
feedback state include robot position, error code, io state, etc.
(see _tm_msgs/msg/FeedbackState.msg_)  
> - publish joint states on _/joint_states_  
> - publish tool pose on _/tool_pose_
>
> __Service Server__
>
> - _/tm_driver/send_script_ (see _tm_msgs/srv/SendScript.srv_) :  
send external script to _Listen node_  
> - _/tm_driver/set_event_ (see _tm_msgs/srv/SetEvent.srv_) :  
send "Stop", "Pause" or "Resume" command to _Listen node_  
> - _/tm_driver/set_io_ (see _tm_msgs/srv/SetIO.srv_) :  
send digital or analog output value to _Listen node_  
> - _/tm_driver/set_position (see _tm_msgs/srv/SetPosition.srv_) :  
send motion command to _Listen node_, the motion type include PTP, LINE, CIRC ans PLINE, the position value is joint angle(__J__) or tool pose(__T__), see [[Expression Editor and Listen Node.pdf]]
>
>


#include "ros/ros.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ros_action/demoAction.h"



int times=0;

int main (int argc, char **argv) {

	ros::init(argc, argv, "demo_action_client");
  /*if(argc != 3){
		ROS_INFO("%d",argc);
		ROS_WARN("Usage: demo_action_client <goal> <time_to_preempt_in_sec>");
		return 1;
	}*/

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ros_action::demoAction> ac("demo_action", true);

  ROS_INFO("Waiting for action server to start.");

  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  

ros_action::demoGoal goal;
  // send a goal to the action
std::cout<<"posizione iniziale : ";
std::cin>>goal.pos_start;
std::cout<<"posizione finale : ";
std::cin>>goal.pos_end;
std::cout<<"velocità max rpm : ";
std::cin>>goal.velocity;
std::cout<<"tempo : ";
std::cin>>times;

 
  ROS_INFO("Starter [%d] , Finish [%d] , Velocity [%d] , Time [%d]",goal.pos_start ,goal.pos_end,goal.velocity , times);
  
	ac.sendGoal(goal);





  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(times));
  //Preempting task
  ac.cancelGoal();

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    //Preempting the process
		ac.cancelGoal();

  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

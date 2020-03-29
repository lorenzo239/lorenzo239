#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h>
#include "ros_action/demoAction.h"
#include <iostream>
#include <sstream>

class action_class {

	private:
  	ros::NodeHandle nh_;
  	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
  	actionlib::SimpleActionServer<ros_action::demoAction> as; 
  	// create messages that are used to published feedback/result
		ros_action::demoFeedback feedback;
		ros_action::demoResult result;

  	std::string action_name;
  	int goal;
  	int progress;
	
	int one_click;

	public:
		action_class(std::string name) :
  	  as(nh_, name, boost::bind(&action_class::executeCB, this, _1), false),
  	  action_name(name) {
				as.registerPreemptCallback(boost::bind(&action_class::preemptCB, this));
				as.start();
  		}
	
		void preemptCB(){
			ROS_WARN("%s got preempted!", action_name.c_str());
			result.pos_finish = progress;
			as.setPreempted(result,"I got Preempted"); 
  	}
  
		void executeCB(const ros_action::demoGoalConstPtr &goal) {
			if(!as.isActive() || as.isPreemptRequested()) return;
			
			one_click=60/(goal->velocity);                //goal-> velocità/6 sono rpm			
			ros::Rate rate(5);
                        
			ROS_INFO("%s is processing the goal %d", action_name.c_str(), goal->pos_end);
			for(progress = goal->pos_start ; progress <= goal->pos_end; progress++){
				//Check for ros
				if (!ros::ok()) {
					result.pos_finish = progress;
					as.setAborted(result,"I failed !");
					ROS_INFO("%s Shutting down",action_name.c_str());
					break;
				}
		
				if(!as.isActive() || as.isPreemptRequested()){
					return;
				}	

				if(goal->pos_end <= progress) {
					ROS_INFO("%s Succeeded at getting to goal %d", action_name.c_str(), goal->pos_end);
					result.pos_finish = progress;
					as.setSucceeded(result);
				}
				else {
					feedback.current_pos = progress;
					ROS_INFO("Setting to goal %d / %d",feedback.current_pos,goal->pos_end);
					
					as.publishFeedback(feedback);
			}
		
			sleep(one_click);
		}	
  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "demo_action");
  ROS_INFO("Starting Demo Action Server");
  action_class demo_action_obj(ros::this_node::getName());
  ros::spin();
  return 0;
}

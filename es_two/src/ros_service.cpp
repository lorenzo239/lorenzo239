#include "ros/ros.h"
#include "es_two/service.h"
#include <iostream>
#include <sstream>


using namespace std;
int fibonacci=0;
int inde=0;
int temp1=0;
int temp2=1; 
int i;
bool service_callback
(es_two::service::Request &req, es_two::service::Response &res) {
 std::stringstream ss;
	
 inde=0;
 inde = atoi(req.in.c_str());
 i=1;
 temp1=0;
 temp2=1; 
  
 do{
 fibonacci=temp1+temp2;
 temp1=temp2;
 temp2=fibonacci;
 i++;}
 while (i<inde);


 ss<<fibonacci;
 res.out=ss.str();
 ROS_INFO("From Client [%s], Server says [%s]", req.in.c_str(), res.out.c_str());
 return true;
 

}
 

int main (int argc, char **argv){
 ros::init(argc, argv, "ros_server");
 ros::NodeHandle n;
 ros::ServiceServer service= n.advertiseService("service", service_callback);
 ROS_INFO("Ready to receive from client.");
 ros::spin();
 return 0;

 

}

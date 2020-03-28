#include "ros/ros.h"
#include "es_three/service.h"
#include <iostream>
#include <sstream>

int uno;
int due;
int somma;


using namespace std;

 bool service_callback
 (es_three::service::Request &req, es_three::service::Response &res) {
 std::stringstream ss;



 uno=atoi(req.numbera.c_str());

 cout <<" numero uno  ----> " <<uno<<endl;


 due=atoi(req.numberb.c_str());

 cout <<" numero due ---->" <<due<<endl;
	

 somma=uno+due;

 ss<<somma;
 res.sum=ss.str();
 ROS_INFO("Server says [%s]", res.sum.c_str());
 cout<<endl;
 return true;
 
}
 

int main (int argc, char **argv){
 ros::init(argc, argv, "nodotwo");
 ros::NodeHandle n;
 ros::ServiceServer service= n.advertiseService("service", service_callback);
 ROS_INFO("Ready to receive from client.");
 ros::spin();
 return 0;

 

}

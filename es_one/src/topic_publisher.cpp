#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

int main (int argc, char **argv){

ros:: init(argc, argv, "topic_publisher");
ros::NodeHandle nh;
ros::Publisher pub= nh.advertise <std_msgs::string>("strings", 10);
ros::Rate rate(10);

while(ros::ok()) {

std::cout<<"Input:";
std::getline(std::cin,msg.data);

ROS_INFO("%s",msg.data.c_str());
pub.publish(msg);

//rate.sleep(); //Do we need to sleep?

}

return 0;
}

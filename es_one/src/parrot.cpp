#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

//subscriber class
class Parrot_Sub{

 public:
 Parrot_Sub();
 void callback(std_msgs::StringContPtr data);

 private:
 ros::NodeHandle _nh;
 ros::Subscriber _topic_sub;

};

//subscribe to the '/string' topic
Parrot_Sub::Parrot_Sub(){
 _topic_sub= _nh.subscribe("/strings", 0, &Parrot_Sub::callback, this);

}

//print received data
void Parrot_Sub::callback(const std_msgs::String::ConstPtr data){
ROS_INFO("Listener: %s", data->data.c_str());

}

int main (int argc, char **argv){
 ros::init(argc, argv, "parrot");
 Parrot_Sub sub;
 ros::spin();
 return 0;

}


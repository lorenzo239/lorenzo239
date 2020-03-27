#include "ros/ros.h"
#include "es_two/service.h"
#include <iostream>
#include <sstream>

using namespace std;
int numero=0;
int numero2=0;
int i=0;

int main (int argc, char **argv){

 ros::init(argc, argv, "fibonacci");
 ros::NodeHandle n;
 ros::Rate loop_rate (10);
 ros::ServiceClient client= n.serviceClient<es_two::service>("service");

while (ros::ok()) {
 es_two::service srv;
 std::stringstream ss;
cout <<"Scrivi il primo numero : ";
cin>>numero;


cout <<"Scrivi il secondo numero : ";
cin>>numero2;

cout<<"   Fibonacci :  ";
i=0;
do {
		
 ss<<numero;
 srv.request.in=ss.str();

 

 if(client.call(srv)) {
 cout<< srv.response.out << "  ";

 }

 else {

 ROS_ERROR ("Failed to call service");
 return 1; 
 }


ss.str("");
numero++;
i++;
}
while (i<numero2);

cout<<endl;

ros::spinOnce();
loop_rate.sleep();
}
return 0;

 

}

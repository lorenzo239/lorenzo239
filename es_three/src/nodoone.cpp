#include "ros/ros.h"
#include "es_three/service.h"
#include "std_msgs/Int32.h"
#include <iostream>


using namespace std;

class valore {

	public :
	int uno;
	int due;

};

                
int a;


int main (int argc, char **argv){

 ros::init(argc, argv, "nodoone");
 ros::NodeHandle n;
 ros::Rate loop_rate (10);
 ros::ServiceClient client= n.serviceClient<es_three::service>("service");

while (ros::ok()) {
 es_three::service srv;

std::stringstream ss;

valore numero;

cout <<"Ecco i due numeri casuali : ";

numero.uno= rand()% 100 +1;
ss<<numero.uno;
srv.request.numbera=ss.str();

cout <<" numero uno  ----> " <<numero.uno<<"  "<<srv.request.numbera<<endl;

ss.str("");


numero.due= rand()% 100 +1;

ss<<numero.due;
srv.request.numberb=ss.str();

cout <<" numero due  ----> " <<numero.due<<"  "<<srv.request.numberb<<endl;

ss.str("");

if(client.call(srv)) {


 
cout<<"Somma dei due numeri : "; 
cout<< srv.response.sum<< "  "<<endl;

cout <<"prossimo, premi invio ";
cin >> a;
}

ros::spinOnce();
loop_rate.sleep();
}
return 0;

 

}

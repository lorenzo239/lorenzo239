#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <actionlib/client/simple_action_client.h>
#include "surveillance_task/navAction.h"

using namespace std;

class SURV_CLIENT {
    public:
        SURV_CLIENT();
        void laser_cb( sensor_msgs::LaserScanConstPtr );
        void odom_cb( nav_msgs::OdometryConstPtr );
        void run();
				void main_loop();

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _laser_sub;
        bool _obs_contact;

};

SURV_CLIENT::SURV_CLIENT() {
    _laser_sub = _nh.subscribe("/laser/scan",0,&SURV_CLIENT::laser_cb,this);
    _odom_sub = _nh.subscribe("/odom", 0, &SURV_CLIENT::odom_cb, this);
    _obs_contact = false;
}


void SURV_CLIENT::laser_cb( sensor_msgs::LaserScanConstPtr laser) {
  int nPoints = (int)(M_PI/laser->angle_increment);
  float minDist=10000;
  for (int i=0; i<nPoints; i++) {
    if (laser->ranges[i]<minDist && laser->ranges[i]>0) {
      minDist = laser->ranges[i];
    }
  }

  if (minDist<1.0) _obs_contact=true;
  else if (minDist>2.0) _obs_contact=false;
}

void SURV_CLIENT::odom_cb( nav_msgs::OdometryConstPtr ) {
    //cout << "Odom!" << endl;
}

//main loop!
void SURV_CLIENT::main_loop() {

  actionlib::SimpleActionClient<surveillance_task::navAction> ac("auto_nav_server", true);
  ac.waitForServer(); //will wait for infinite time


	std::vector<geometry_msgs::Point> points;
  points.resize(4);
  points[0].x = 1;
  points[0].y = 0;

  points[1].x = 1;
  points[1].y = 1;

  points[2].x = 0;
  points[2].y = 1;

  points[3].x = 0;
  points[3].y = 0;


	surveillance_task::navGoal g;
	ros::Rate r(10);

  while(ros::ok()) {

    for (int i=0; i< points.size(); i++) {

    	bool done = false;
      g.x_dest = points[i].x;
    	g.y_dest = points[i].y;
      ac.sendGoal(g);

    	while ( !done ) {
    		if ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED ) {
    			done = true;
    		}
    		r.sleep();
    	}


    	if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) cout << "Target position reached!" << endl;
    	else {
        cout << "Target position NOT reached! Manual intervention needed." << endl;
        while (_obs_contact) r.sleep();
        cout << "Automatic navigation restarted!"<<endl;
        i--;
    	}

    }
  }


}



void SURV_CLIENT::run() {
   	boost::thread ctrl_loop_t( &SURV_CLIENT::main_loop, this );
    ros::spin();
}



int main( int argc, char** argv) {

    ros::init(argc, argv, "surv_client" );
    SURV_CLIENT s;
    s.run();

    return 0;

}

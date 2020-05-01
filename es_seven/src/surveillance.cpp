#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <actionlib/server/simple_action_server.h>
#include "surveillance_task/navAction.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>


using namespace std;

class SURV {
    public:
        SURV(std::string name);
        void laser_cb( sensor_msgs::LaserScanConstPtr );
        void odom_cb( nav_msgs::OdometryConstPtr );
        void run();
        void executeCB( const surveillance_task::navGoalConstPtr &goal );

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _laser_sub;
        actionlib::SimpleActionServer<surveillance_task::navAction> _as;
        string action_name;
        ros::Publisher _cmd_vel_pub;
        double _yaw;
        float _x;
        float _y;
        bool _obs_contact;

};

SURV::SURV(std::string name) :
        _as(_nh, name, boost::bind(&SURV::executeCB, this, _1), false),
          action_name(name) {
    _laser_sub = _nh.subscribe("/laser/scan",0,&SURV::laser_cb,this);
    _odom_sub = _nh.subscribe("/odom", 0, &SURV::odom_cb, this);
    _cmd_vel_pub = _nh.advertise< geometry_msgs::Twist>("/cmd_vel", 0);
    _as.start();
    _obs_contact=false;

}



void SURV::executeCB( const surveillance_task::navGoalConstPtr &goal ) {
    bool done = false;
    float kp_o = 0.3;
    float kp_p = 0.2;
    double z_vel,x_vel;
    geometry_msgs::Twist cmd;
    ros::Rate r(10);

    while (!done && !_as.isPreemptRequested() && _as.isActive()) {

        double des_yaw = atan2( goal->y_dest - _y, goal->x_dest - _x);
        double yaw_e = des_yaw - _yaw;
        if(fabs(yaw_e) > M_PI)
            yaw_e = yaw_e - 2*M_PI* ((yaw_e>0)?1:-1);

        //if (yaw_e>M_PI) yaw_e= -(yaw_e-M_PI);
        //else if (yaw_e < -M_PI) yaw_e= -(yaw_e+M_PI);

        double pos_e = sqrt((goal->x_dest - _x)*(goal->x_dest - _x)+(goal->y_dest - _y)*(goal->y_dest - _y));
    		cout << "yaw_e: " << yaw_e << endl;
        if( fabs(yaw_e) > 0.1 ) {
               //control orientation
               z_vel = -kp_o*yaw_e;
            //   cout<< "z_vel: "<<z_vel<<endl;
        }
        else {

              //control orientation and position
              x_vel = kp_p*pos_e;
              z_vel = -kp_o*yaw_e;
            //  cout<< "x_vel: "<<x_vel<<" "<<"z_vel: "<<z_vel<<endl;
        }

        if( (fabs(yaw_e) < 0.02) && (fabs(pos_e) < 0.05)) {
          done = true;
          _as.setSucceeded();
        }

        if (_obs_contact) {
          _obs_contact=false;
          z_vel=0;
          x_vel=0;
          _as.setPreempted();
        }

        cmd.angular.z = z_vel;
        cmd.linear.x = x_vel;
        _cmd_vel_pub.publish(  cmd );

        r.sleep();
    }
}


void SURV::laser_cb( sensor_msgs::LaserScanConstPtr laser) {
  int nPoints = (int)(M_PI/laser->angle_increment);
  float minDist=10000;
  for (int i=0; i<nPoints; i++) {
    if (laser->ranges[i]<minDist && laser->ranges[i]>0) {
      minDist = laser->ranges[i];
      //cout<<i<<endl;
    }
  }
  //cout<<minDist<<endl;
  if (minDist<1.0) _obs_contact=true;
  else if (minDist>2.0) _obs_contact=false;

}

void SURV::odom_cb( nav_msgs::OdometryConstPtr odom ) {

    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _yaw);

    _x = odom->pose.pose.position.x;
    _y = odom->pose.pose.position.y;

    //cout<< "x: "<<_x<<" "<<"y: "<<_y<<endl;

}

//main loop!



void SURV::run() {
    ros::spin();
}



int main( int argc, char** argv) {

    ros::init(argc, argv, "surv" );
    SURV s("auto_nav_server");
    s.run();

    return 0;

}

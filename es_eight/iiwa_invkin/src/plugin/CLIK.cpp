#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>

using namespace std;

	
namespace gazebo
{
  class ClikPlugin : public ModelPlugin
  {

  private:
  	ros::NodeHandle* _nh;
	physics::ModelPtr model;

	ros::Subscriber _destination_sub;


	KDL::Tree iiwa_tree;
	
	KDL::ChainJntToJacSolver *_jacsolver;
	KDL::ChainFkSolverPos_recursive *_fksolver;
	KDL::ChainIkSolverVel_pinv *_ik_solver_vel;
	KDL::ChainIkSolverPos_NR *_ik_solver_pos;

	KDL::Chain _k_chain;

	ros::Subscriber _js_sub;
	KDL::JntArray *_q_in;
	bool _first_js;
	bool _first_fk;
	ros::Publisher _cmd_pub[7];
	KDL::Frame _p_out;

	int DELTA;
	int TEMPOFINALE;
	std::vector<KDL::Frame> timeseries;

	public: 
		ClikPlugin();
		void Load(physics::ModelPtr, sdf::ElementPtr);
		bool init_robot_model();
		void get_dirkin();
		void joint_states_cb( sensor_msgs::JointState );
		void ctrl_loop();
		void traj_calc_cb( geometry_msgs::Point );
};


ClikPlugin::ClikPlugin(){
	_nh = new ros::NodeHandle();	

	_cmd_pub[0] = _nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint1_position_controller/command", 0);
	_cmd_pub[1] = _nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint2_position_controller/command", 0);
	_cmd_pub[2] = _nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint3_position_controller/command", 0);
	_cmd_pub[3] = _nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint4_position_controller/command", 0);
	_cmd_pub[4] = _nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint5_position_controller/command", 0);
	_cmd_pub[5] = _nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint6_position_controller/command", 0);
	_cmd_pub[6] = _nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint7_position_controller/command", 0);

	_js_sub = _nh->subscribe("/lbr_iiwa/joint_states", 0, &ClikPlugin::joint_states_cb, this);
	_destination_sub = _nh->subscribe("/destination", 0, &ClikPlugin::traj_calc_cb, this);


	if (!init_robot_model()) exit(1); 

	_first_js = false;
	_first_fk = false;

	DELTA = 10;
	TEMPOFINALE = 5;

	timeseries.resize((DELTA*TEMPOFINALE)+1);


}

bool ClikPlugin::init_robot_model() {
	std::string robot_desc_string;

	_nh->param("robot_description", robot_desc_string, std::string());

	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	std::string base_link = "lbr_iiwa_link_0";
	std::string tip_link  = "lbr_iiwa_link_7";

	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;

	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );
	_ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 );

	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );

	return true;
}


void ClikPlugin::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<7; i++ ) 
		_q_in->data[i] = js.position[i];

	_first_js = true;
}

void ClikPlugin::traj_calc_cb(geometry_msgs::Point dest){

	// Check se il punto è accettabile
	//...


	//Creazione vettore
	get_dirkin();  //ottiene _p_out (posizione attuale)

	float lung_x;
	float lung_y;
	float lung_z;

	lung_x = dest.x - _p_out.p.x();
	lung_y = dest.y - _p_out.p.y();
	lung_z = dest.z - _p_out.p.z();

	timeseries[0].p.data[0] = _p_out.p.data[0];
	timeseries[0].p.data[1] = _p_out.p.data[1];
	timeseries[0].p.data[2] = _p_out.p.data[2];


	for(int i=1;i<((TEMPOFINALE*DELTA)+1);i++){

		timeseries[i].p.data[0] = timeseries[i-1].p.data[0] + (lung_x/(TEMPOFINALE*DELTA));
		timeseries[i].p.data[1] = timeseries[i-1].p.data[1] + (lung_y/(TEMPOFINALE*DELTA));
		timeseries[i].p.data[2] = timeseries[i-1].p.data[2] + (lung_z/(TEMPOFINALE*DELTA));


		double qx, qy, qz, qw;
		_p_out.M.GetQuaternion( qx, qy, qz, qw);
	}

	ctrl_loop();

}


void ClikPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {	
	cout<<"load OK"<<endl;
}


void ClikPlugin::get_dirkin() {

	while( !_first_js ) usleep(0.1);

	_fksolver->JntToCart(*_q_in, _p_out);

	_first_fk = true;
	
}



void ClikPlugin::ctrl_loop() {
	
	std_msgs::Float64 cmd[7];
	KDL::JntArray q_out(*_q_in);
	KDL::JntArray qd_out(_k_chain.getNrOfJoints());
	float GAIN = 10.0;
	double u_x,u_y,u_z;
	double xd_k = 0.0;
	double yd_k = 0.0;
	double zd_k = 0.0;
	double threshold = 0.01;
	KDL::Frame F_dest;
	KDL::Twist twist;

	ros::Rate r(DELTA*TEMPOFINALE);


	KDL::SetToZero(twist.rot);
	KDL::SetToZero(twist.vel);



	for(int i=1;i<((TEMPOFINALE*DELTA)+1);i++){
		
		get_dirkin();


		if(abs(timeseries[i].p.data[0] - _p_out.p.data[0]) < threshold){
			xd_k = 0;
		}else{
			xd_k = 0.1;
		}

		if(abs(timeseries[i].p.data[1] - _p_out.p.data[1]) < threshold){
			yd_k = 0;
		}else{
			yd_k = 0.1;
		}

		if(abs(timeseries[i].p.data[2] - _p_out.p.data[2]) < threshold){
			zd_k = 0;
		}else{
			zd_k = 0.1;
		}

		
		//OPEN LOOP 
		/*if( _ik_solver_pos->CartToJnt(*_q_in, timeseries[i], q_out) != KDL::SolverI::E_NOERROR ) 
			cout << "failing in ik!" << endl;*/


		///CLOSED LOOP-----------
		u_x = xd_k + GAIN*(timeseries[i].p.data[0] - _p_out.p.data[0]);
		u_y = yd_k + GAIN*(timeseries[i].p.data[1] - _p_out.p.data[1]);
		u_z = zd_k + GAIN*(timeseries[i].p.data[2] - _p_out.p.data[2]);


		if( _ik_solver_pos->CartToJnt(*_q_in, timeseries[i], q_out) != KDL::SolverI::E_NOERROR ) 
			cout << "failing in ik!";

		twist.vel.data[0]=u_x;
		twist.vel.data[1]=u_y;
		twist.vel.data[2]=u_z;
		
		if( _ik_solver_vel->CartToJnt(*_q_in, twist, qd_out) != KDL::SolverI::E_NOERROR ) 
			cout << "failing in ik!" << endl;

		//EULERO
		for(int i=0; i<7; i++) {
			q_out.data[i] = q_out.data[i] + (qd_out.data[i] * 0.1);
		}
		//----------------------------


		for(int i=0; i<7; i++) {
			cmd[i].data = q_out.data[i];
		}
		for(int i=0; i<7; i++) {
			_cmd_pub[i].publish (cmd[i]);
		}
		


		//Debug---------------------------------

		cout<<"----------"<<endl;
		cout<<"PASSO: "<<i<<endl;

		cout<<"x_k: "<<std::to_string(timeseries[i].p.data[0])<<"  ";
		cout<<"xd_k: "<<std::to_string(xd_k)<<" ";
		cout<<"dist_x: "<<abs(timeseries[i].p.data[0] - _p_out.p.data[0])<<" ";
		cout<<"u_x: "<<std::to_string(u_x)<<endl;

		cout<<"y_k: "<<std::to_string(timeseries[i].p.data[1])<<"  ";
		cout<<"yd_k: "<<std::to_string(yd_k)<<" ";
		cout<<"dist_y: "<<abs(timeseries[i].p.data[1] - _p_out.p.data[1])<<" ";
		cout<<"u_y: "<<std::to_string(u_y)<<endl;
		
		cout<<"z_k: "<<std::to_string(timeseries[i].p.data[2])<<"  ";
		cout<<"zd_k: "<<std::to_string(zd_k)<<" ";
		cout<<"dist_z: "<<abs(timeseries[i].p.data[2] - _p_out.p.data[2])<<" ";
		cout<<"u_z: "<<std::to_string(u_z)<<endl;

		//--------------------------------------

		r.sleep();

	}

}


  GZ_REGISTER_MODEL_PLUGIN(ClikPlugin)
}

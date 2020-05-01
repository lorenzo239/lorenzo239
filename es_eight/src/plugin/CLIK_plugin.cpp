#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/Float64.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "geometry_msgs/Point.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

using namespace Eigen;
using namespace std;

namespace gazebo
{
  class CLIKPlugin : public ModelPlugin
  {

	private: ros::NodeHandle _nh;
	private: physics::ModelPtr model;
	private: event::ConnectionPtr updateConnection;
	private: physics::JointPtr _joints[7];
	private: KDL::JntArray *_q_in;
  private:
     ros::Subscriber _des_pos_sub;
     geometry_msgs::Point _des_pos;
     ros::Publisher _cmd_pub[7];
     ros::Publisher _debug;
     KDL::ChainFkSolverPos_recursive *_fksolver; //Forward position solver
     KDL::ChainJntToJacSolver *_jacobian_solver; //Jacobian solver
     KDL::Frame _p_out;
     KDL::Jacobian  *_J;            // Jacobian
     KDL::Chain _k_chain;
     KDL::Tree iiwa_tree;
     bool _get_pos;
  public: void ctrl_loop();
  public: bool init_robot_model();
  public: void des_pos_cb( geometry_msgs::PointConstPtr );
	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		//_nh = new ros::NodeHandle();
		model = _parent;
    _get_pos = false;
    cout<<"CLIK plugin started!"<<endl;
    gzdbg << "CLIK plugin started!" << std::endl;
    ROS_INFO("CLIK plugin started!");
    _des_pos_sub = _nh.subscribe("/des_pos", 0, &CLIKPlugin::des_pos_cb, this);

    _cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint1_position_controller/command", 0);
  	_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint2_position_controller/command", 0);
  	_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint3_position_controller/command", 0);
  	_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint4_position_controller/command", 0);
  	_cmd_pub[4] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint5_position_controller/command", 0);
  	_cmd_pub[5] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint6_position_controller/command", 0);
  	_cmd_pub[6] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint7_position_controller/command", 0);
    _debug = _nh.advertise< std_msgs::Float32MultiArray > ("/lbr_iiwa/debug", 0);

/*    std_msgs::Float64 cmd;
    cmd.data = 1.0;
    for(int i=0; i<7; i++) {
      _cmd_pub[i].publish(cmd);
    } */

    for (int i=0; i<7; i++) {
      char buffer[50];
      sprintf(buffer, "lbr_iiwa_joint_%d", (i+1));
		  _joints[i] = this->model->GetJoint(buffer);
    }
    //_joints[0] = this->model->GetJoint("lbr_iiwa_joint_1");

		//_w_v_pub = _nh->advertise< std_msgs::Float32MultiArray >("/diffbuffer_wheels/vel", 0);
		//_js_positions.data.resize(7);
    init_robot_model();
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CLIKPlugin::OnUpdate, this));
    boost::thread ctrl_loop_t ( &CLIKPlugin::ctrl_loop, this);
  	//ros::spin();

	}

    // Called by the world update start event
    public: void OnUpdate()  {
      for (int i=0; i<7; i++)
  		  _q_in->data[i]=_joints[i]->Position(0);
        //_q_in->data[0]=_joints[0]->Position(0);
    }

  };

  void CLIKPlugin::des_pos_cb( geometry_msgs::PointConstPtr pos ) {
    _des_pos.x = pos->x;
    _des_pos.y = pos->y;
    _des_pos.z = pos->z;
    _get_pos = true;
  }

  void CLIKPlugin::ctrl_loop() {
    std_msgs::Float64 cmd[7];
    std_msgs::Float64 debug_msg;
    std_msgs::Float32MultiArray  debug_msg1;
    KDL::JntArray q_out(_k_chain.getNrOfJoints());
    KDL::JntArray q_dot_out(_k_chain.getNrOfJoints());
    ros::Rate r(100);

    Vector3d error( 0 , 0 , 0 );
    Vector3d c_action( 0 , 0 , 0 );
    VectorXd q_dot(7);
    VectorXd q(7);
    q<<0,0,0,0,0,0,0;
    MatrixXd jacob(6,7);
    MatrixXd jac_transpose(7,3);
    float Kp = 10;

    debug_msg1.data.resize(3*7);

    while(!_get_pos) usleep(1);

    while( ros::ok() ) {

      _fksolver->JntToCart(*_q_in, _p_out);

      if (!_get_pos) {
        _des_pos.x = _p_out.p.x();
        _des_pos.y = _p_out.p.y();
        _des_pos.z = _p_out.p.z();
        _get_pos = true;
      }


    	error(0) = _des_pos.x - _p_out.p.x();
    	error(1) = _des_pos.y - _p_out.p.y();
    	error(2) = _des_pos.z - _p_out.p.z();
      c_action = Kp*error;

      _jacobian_solver->JntToJac(*_q_in, *_J);
      jacob = _J->data.block(0,0,3,7);
      jac_transpose = jacob.transpose();
      int k=0;
      for (int i = 0; i<7; i++)
        for (int j=0; j<3; j++, k++)
          debug_msg1.data[k]=jac_transpose(i,j);

      _debug.publish(debug_msg1);

      q_dot = jac_transpose*c_action;
      //q_dot << 0.1,0.1,0.1,0.1,0.1,0.1,0.1;
      //debug_msg.data = q_dot(4);
      //_debug.publish(debug_msg);
      q = q + 0.01*q_dot;

  		for(int i=0; i<7; i++) {
  			cmd[i].data = q(i);
  		}
  		for(int i=0; i<7; i++) {
  			_cmd_pub[i].publish(cmd[i]);
  		}

  		r.sleep();
  	}
  }

  bool CLIKPlugin::init_robot_model() {
  	std::string robot_desc_string;
  	_nh.param("robot_description", robot_desc_string, std::string());
  	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
  		ROS_ERROR("Failed to construct kdl tree");
  		return false;
  	}

  	std::string base_link = "lbr_iiwa_link_0";
  	std::string tip_link  = "lbr_iiwa_link_7";
  	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;
  	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );
    _jacobian_solver = new KDL::ChainJntToJacSolver(_k_chain);

  	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
    _J = new KDL::Jacobian( _k_chain.getNrOfJoints() );
  	return true;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CLIKPlugin)
}

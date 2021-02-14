#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Eigen/Geometry"

#include <kdl_parser/kdl_parser.hpp>

ros::NodeHandle n;

void get_shoulder_lift_position(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->position.c_str());
}

//Parse urdf model and generate KDL tree
KDL::Tree my_tree;
if (!kdl_parser::treeFromFile("filename", my_tree)){
   ROS_ERROR("Failed to construct kdl tree");
   return false;
}

std::string root = "base_link";
std::string tip = "wrist_3_link";

KDL::Chain chain; 
my_tree.getChain(root, tip, chain);

int Joints = 6;

float Kp = 100.0
float Kd = 1.0

//Cartesian position
KDL::Frame    x_cart;

Eigen::Matrix<double, Joints, 1> joint_pos;
Eigen::Matrix<double, Joints, 1> joint_vel;

Eigen::Matrix<double, 6, 1> xdot;

Eigen::Matrix<double, 6, Joints> jacobian;

Eigen::Matrix<double, 6, 1> F;
Eigen::Matrix<double, Joints, 1> tau;
	
// Startup
KDL::ChainFkSolverPos *fk_solver = new KDL::ChainFkSolverPos(chain);
KDL::ChainJntToJacSolver *jnt_to_jac_solver = new KDL::ChainJntToJacSolver(chain); 
//get joint positions (from state publisher?)

//KDL to Eigen

// Update
float dt = 0.001; //(or computed)

ros::Subscriber sub = n.subscribe("/shoulder_lift_joint_position_controller/state", 1000, get_shoulder_lift_position);
ros::Publisher joint_com_pub = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
//get joint positions (from state publisher?)
//get joint velocity(from state publisher?)

// joint positions Eigen to KDL

fk_solver->JntToCart(...;
//jnt_to_jac_solver->JntToJac(...);

// joint pos + jacobian KDL to Eigen

//xdot = jacobian * joint_velocity;

//compute desired next step in path
//...

//compute cartesian error

//Calculate cartesian force

//tau = J.transpose() * F;


std_msgs::Float64 position;
position = 1.5;
//sendo torque to the joints
joint_com_pub.publish(position);




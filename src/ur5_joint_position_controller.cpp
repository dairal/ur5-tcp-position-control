#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
//#include <eigen_conversions/eigen_kdl.h>

#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/JointControllerState.h"

#include "Eigen/Geometry"

std::string joint_strings[] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

const int Joints = 6;

Eigen::Matrix<double, Joints, 1> joint_pos;
Eigen::Matrix<int, Joints, 1> joint_state_mapping;
int joint_mapping_init_flag = 0;

void get_joint_states(const sensor_msgs::JointState::ConstPtr& state_msg) {
	
	if(!joint_mapping_init_flag) {
		for(int i=0; i<Joints; i++) {
			for(int j=0; j<Joints; j++) {
				if(joint_strings[i] == state_msg->name[j]) {
					joint_state_mapping[i] = j;
					break;
				}
			}
		}
		joint_mapping_init_flag = 1;
	}

	for(int i=0; i<Joints; i++) {
		joint_pos(i,0) = state_msg->position[joint_state_mapping[i]];
		//ROS_INFO("Position %d: %f", i, joint_pos(i,0));
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jnt_ctr");

	ros::NodeHandle n;

	ros::Rate loop_rate(20000);

	ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1000, get_joint_states);

	ros::Publisher joint_com_pub[6]; 
	joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_effort_controller/command", 1000);
	joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_effort_controller/command", 1000);
	joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_effort_controller/command", 1000);
	joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_effort_controller/command", 1000);
	joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_effort_controller/command", 1000);
	joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_effort_controller/command", 1000);

	double Kp[] = {1, 20, 10, 3, 2, 2};
	double Ki[] = {0.00, 0.0, 0.0, 0.0, 0.0, 0.0};
	double Kd[] = {100.0, 1.0, 1.0, 100.0, 100.0, 5.0};

	Eigen::Matrix<double, Joints, 1> target_joint_pos;
	target_joint_pos << 0.8, -0.5, 1.0, 1.0, 1.0, 1.0; 

	Eigen::Matrix<double, Joints, 1> prev_joint_pos = joint_pos;
	double i_out[] = {0, 0, 0, 0, 0, 0};

	while(ros::ok()) {

		for(int i=0; i<Joints; i++) {
			 
			double pos_error = (target_joint_pos(i,0) - joint_pos(i,0));
			ROS_INFO("Position error joint %d: %f - %f = %f", i, target_joint_pos(i,0), joint_pos(i,0), pos_error);
			//ROS_INFO("Pos error joint %d: %f", i, pos_error);
			double p_out = Kp[i] * pos_error;
			i_out[i] += Ki[i] * pos_error;
			double d_out = Kd[i] * (joint_pos(i,0) - prev_joint_pos(i,0));

			// ROS_INFO("P out joint %d: %f", i, p_out);
			// ROS_INFO("I out joint %d: %f", i, i_out);
			// ROS_INFO("D out joint %d: %f", i, d_out);

			double control_out = p_out + i_out[i] - d_out;
			std_msgs::Float64 pub_msg;
			pub_msg.data = control_out;

			joint_com_pub[i].publish(pub_msg);
			prev_joint_pos = joint_pos;

			ROS_INFO("Control out joint %d: %f", i, control_out);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
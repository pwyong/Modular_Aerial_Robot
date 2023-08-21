#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <chrono>

#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Imu.h>

#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>

#include "nav_msgs/Odometry.h"

ros::Publisher data_log_publisher;

std_msgs::Float64MultiArray data_log;

geometry_msgs::Vector3 position;
geometry_msgs::Vector3 desired_position;
geometry_msgs::Vector3 attitude;
geometry_msgs::Vector3 desired_attitude;
geometry_msgs::Vector3 linear_velocity;
geometry_msgs::Vector3 desired_linear_velocity;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 desired_force;
geometry_msgs::Vector3 desired_torque;
geometry_msgs::Vector3 center_of_mass;
geometry_msgs::Vector3 bias_gradient;
geometry_msgs::Vector3 filtered_bias_gradient;
geometry_msgs::Vector3 attitude_dob_disturbance;
geometry_msgs::Vector3 external_force;
geometry_msgs::Vector3 external_torque;
geometry_msgs::Vector3 reference_position;
geometry_msgs::Vector3 calculated_force;
geometry_msgs::Vector3 non_bias_external_force;

double PWM_cmd[8]={1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000.};
double individual_motor_thrust[4]={0.0, 0.0, 0.0, 0.0};
double battery_voltage=16.0;
double delta_t=0.0;
double SBUS[10]={1000.,1000.,1000.,1000.,1000.,1000.,1000.,1000.,1000.,1000.};
double theta1=0.0; 
double theta2=0.0;
double desired_theta1=0.0;
double desired_theta2=0.0;
double mhe_delta_t=0.0;

void pos_callback(const geometry_msgs::Vector3& msg);
void desired_pos_callback(const geometry_msgs::Vector3& msg);
void attitude_callback(const geometry_msgs::Vector3& msg);
void desired_attitude_callback(const geometry_msgs::Vector3& msg);
void linear_velocity_callback(const geometry_msgs::Vector3& msg);
void desired_linear_velocity_callback(const geometry_msgs::Vector3& msg);
void pwm_cmd_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
void motor_thrust_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void servo_angle_callback(const sensor_msgs::JointState& msg);
void desired_servo_angle_callback(const sensor_msgs::JointState& msg);
void battery_voltage_callback(const std_msgs::Float32& msg);
void sampling_time_callback(const std_msgs::Float32& msg);
void desired_force_callback(const geometry_msgs::Vector3& msg);
void desired_torque_callback(const geometry_msgs::Vector3& msg);
void center_of_mass_callback(const geometry_msgs::Vector3& msg);
void bias_gradient_callback(const geometry_msgs::Vector3& msg);
void filtered_bias_gradient_callback(const geometry_msgs::Vector3& msg);
void sbus_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
void angular_velocity_callback(const geometry_msgs::Vector3& msg);
void attitude_dob_disturbance_callback(const geometry_msgs::Vector3& msg);
void external_force_callback(const geometry_msgs::Vector3& msg);
void external_torque_callback(const geometry_msgs::Vector3& msg);
void mhe_delta_t_callback(const std_msgs::Float32& msg);
void reference_position_callback(const geometry_msgs::Vector3& msg);
void calculated_force_callback(const geometry_msgs::Vector3& msg);
void non_bias_external_force_callback(const geometry_msgs::Vector3& msg);
void publisherSet();


int main(int argc, char **argv)
{
	ros::init(argc, argv,"data_logging_node");
	
	ros::NodeHandle nh;
	ros::Subscriber attitude_log=nh.subscribe("/angle",1,attitude_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_attitude_log=nh.subscribe("/desired_angle",1,desired_attitude_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber position_log=nh.subscribe("/pos",1,pos_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_position_log=nh.subscribe("/pos_d",1,desired_pos_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber servo_angle_log=nh.subscribe("/joint_states",1,servo_angle_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_servo_angle_log=nh.subscribe("/goal_dynamixel_position",1,desired_servo_angle_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber pwm_log=nh.subscribe("/PWMs",1,pwm_cmd_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_torque_log=nh.subscribe("/torque_d",1,desired_torque_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_force_log=nh.subscribe("/force_d",1,desired_force_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber motor_thrust_log=nh.subscribe("/Forces",1,motor_thrust_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber sbus_log=nh.subscribe("/sbus",1,sbus_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber delta_t_log=nh.subscribe("/delta_t",1,sampling_time_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber battery_voltage_log=nh.subscribe("/battery_voltage",1,battery_voltage_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber linear_velocity_log=nh.subscribe("/lin_vel",1,linear_velocity_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_linear_velocity_log=nh.subscribe("/lin_vel_d",1,desired_linear_velocity_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber Center_of_Mass_log=nh.subscribe("/Center_of_Mass",1,center_of_mass_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber bias_gradient_log=nh.subscribe("/bias_gradient",1,bias_gradient_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber filtered_bias_gradient_log=nh.subscribe("/filtered_bias_gradient",1,filtered_bias_gradient_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber angular_velocity_log=nh.subscribe("/angular_velocity",1,angular_velocity_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber attitude_dob_disturbance_log=nh.subscribe("/att_dhat",1,attitude_dob_disturbance_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber external_force_log=nh.subscribe("/external_force",1,external_force_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber external_torque_log=nh.subscribe("/external_torque",1,external_torque_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber mhe_delta_t_log=nh.subscribe("/mhe_delta_t",1,mhe_delta_t_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber reference_position_log=nh.subscribe("/reference_position",1,reference_position_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber calculated_force_log=nh.subscribe("/calculated_force",1,calculated_force_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber non_bias_external_force_log=nh.subscribe("/non_bias_external_force",1,non_bias_external_force_callback, ros::TransportHints().tcpNoDelay());

	data_log_publisher=nh.advertise<std_msgs::Float64MultiArray>("data_log",10);
	ros::Timer timerPulish_log=nh.createTimer(ros::Duration(1.0/200.0), std::bind(publisherSet));

	ros::spin();
	return 0;
}

void publisherSet()
{
	data_log.data.resize(82);

	data_log.data[0]=attitude.x;
	data_log.data[1]=attitude.y;
	data_log.data[2]=attitude.z;
	data_log.data[3]=desired_attitude.x;
	data_log.data[4]=desired_attitude.y;
	data_log.data[5]=desired_attitude.z;
	data_log.data[6]=position.x;
	data_log.data[7]=position.y;
	data_log.data[8]=position.z;
	data_log.data[9]=desired_position.x;
	data_log.data[10]=desired_position.y;
	data_log.data[11]=desired_position.z;
	data_log.data[12]=theta1;
	data_log.data[13]=theta2;
	data_log.data[14]=desired_theta1;
	data_log.data[15]=desired_theta2;
	data_log.data[16]=PWM_cmd[0];
	data_log.data[17]=PWM_cmd[1];
	data_log.data[18]=PWM_cmd[2];
	data_log.data[19]=PWM_cmd[3];
	data_log.data[20]=PWM_cmd[4];
	data_log.data[21]=PWM_cmd[5];
	data_log.data[22]=PWM_cmd[6];
	data_log.data[23]=PWM_cmd[7];
	data_log.data[24]=desired_torque.x;
	data_log.data[25]=desired_torque.y;
	data_log.data[26]=desired_torque.z;
	data_log.data[27]=desired_force.x;
	data_log.data[28]=desired_force.y;
	data_log.data[29]=desired_force.z;
	data_log.data[30]=individual_motor_thrust[0];
	data_log.data[31]=individual_motor_thrust[1];
	data_log.data[32]=individual_motor_thrust[2];
	data_log.data[33]=individual_motor_thrust[3];
	data_log.data[34]=SBUS[0];
	data_log.data[35]=SBUS[1];
	data_log.data[36]=SBUS[2];
	data_log.data[37]=SBUS[3];
	data_log.data[38]=SBUS[4];
	data_log.data[39]=SBUS[5];
	data_log.data[40]=SBUS[6];
	data_log.data[41]=SBUS[8];
	data_log.data[42]=SBUS[9];
	data_log.data[43]=delta_t;
	data_log.data[44]=battery_voltage;
	data_log.data[45]=linear_velocity.x;
	data_log.data[46]=linear_velocity.y;
	data_log.data[47]=linear_velocity.z;
	data_log.data[48]=desired_linear_velocity.x;
	data_log.data[49]=desired_linear_velocity.y;
	data_log.data[50]=desired_linear_velocity.z;
	data_log.data[51]=center_of_mass.x;
	data_log.data[52]=center_of_mass.y;
	data_log.data[53]=center_of_mass.z;
	data_log.data[54]=bias_gradient.x;
	data_log.data[55]=bias_gradient.y;
	data_log.data[56]=bias_gradient.z;
	data_log.data[57]=filtered_bias_gradient.x;
	data_log.data[58]=filtered_bias_gradient.y;
	data_log.data[59]=filtered_bias_gradient.z;
	data_log.data[60]=angular_velocity.x;
	data_log.data[61]=angular_velocity.y;
	data_log.data[62]=angular_velocity.z;
	data_log.data[63]=attitude_dob_disturbance.x;
	data_log.data[64]=attitude_dob_disturbance.y;
	data_log.data[65]=attitude_dob_disturbance.z;
	data_log.data[66]=external_force.x;
	data_log.data[67]=external_force.y;
	data_log.data[68]=external_force.z;
	data_log.data[69]=external_torque.x;
	data_log.data[70]=external_torque.y;
	data_log.data[71]=external_torque.z;
	data_log.data[72]=mhe_delta_t;	
	data_log.data[73]=reference_position.x;
	data_log.data[74]=reference_position.y;
	data_log.data[75]=reference_position.z;
	data_log.data[76]=calculated_force.x;
	data_log.data[77]=calculated_force.y;
	data_log.data[78]=calculated_force.z;
	data_log.data[79]=non_bias_external_force.x;	
	data_log.data[80]=non_bias_external_force.y;	
	data_log.data[81]=non_bias_external_force.z;	

	data_log_publisher.publish(data_log);
}

void pos_callback(const geometry_msgs::Vector3& msg){
	position.x=msg.x;
	position.y=msg.y;
	position.z=msg.z;
}

void desired_pos_callback(const geometry_msgs::Vector3& msg){
	desired_position.x=msg.x;
	desired_position.y=msg.y;
	desired_position.z=msg.z;
}

void attitude_callback(const geometry_msgs::Vector3& msg){
	attitude.x=msg.x;
	attitude.y=msg.y;
	attitude.z=msg.z;
}

void desired_attitude_callback(const geometry_msgs::Vector3& msg){
	desired_attitude.x=msg.x;
	desired_attitude.y=msg.y;
	desired_attitude.z=msg.z;
}

void servo_angle_callback(const sensor_msgs::JointState& msg){
	theta1=msg.position[0];
	theta2=-msg.position[1];
}

void desired_servo_angle_callback(const sensor_msgs::JointState& msg){
	desired_theta1=msg.position[0];
	desired_theta2=-msg.position[1];
}

void pwm_cmd_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
	for(int i=0;i<8;i++){
		PWM_cmd[i]=msg->data[i];
	}
}

void motor_thrust_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	for(int i=0;i<4;i++){
		individual_motor_thrust[i]=msg->data[i];
	}	
}

void linear_velocity_callback(const geometry_msgs::Vector3& msg){
	linear_velocity.x=msg.x;
	linear_velocity.y=msg.y;
	linear_velocity.z=msg.z;
}


void desired_linear_velocity_callback(const geometry_msgs::Vector3& msg){
	desired_linear_velocity.x=msg.x;
	desired_linear_velocity.y=msg.y;
}

void battery_voltage_callback(const std_msgs::Float32& msg){
	battery_voltage=msg.data;	
}

void sampling_time_callback(const std_msgs::Float32& msg){
	delta_t=msg.data;
}

void desired_force_callback(const geometry_msgs::Vector3& msg){
	desired_force.x=msg.x;
	desired_force.y=msg.y;
	desired_force.z=msg.z;
}

void desired_torque_callback(const geometry_msgs::Vector3& msg){
	desired_torque.x=msg.x;
	desired_torque.y=msg.y;
	desired_torque.z=msg.z;
}

void center_of_mass_callback(const geometry_msgs::Vector3& msg){
	center_of_mass.x=msg.x;
	center_of_mass.y=msg.y;
	center_of_mass.z=msg.z;
}

void bias_gradient_callback(const geometry_msgs::Vector3& msg){
	bias_gradient.x=msg.x;
	bias_gradient.y=msg.y;
	bias_gradient.z=msg.z;
}

void filtered_bias_gradient_callback(const geometry_msgs::Vector3& msg){
	filtered_bias_gradient.x=msg.x;
	filtered_bias_gradient.y=msg.y;
	filtered_bias_gradient.z=msg.z;
}

void sbus_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
	for(int i=0;i<10;i++){
		SBUS[i]=msg->data[i];	
	}
}

void angular_velocity_callback(const geometry_msgs::Vector3& msg){
	angular_velocity=msg;
}

void attitude_dob_disturbance_callback(const geometry_msgs::Vector3& msg){
	attitude_dob_disturbance=msg;
}

void external_force_callback(const geometry_msgs::Vector3& msg){
	external_force=msg;
}

void external_torque_callback(const geometry_msgs::Vector3& msg){
	external_torque=msg;
}

void mhe_delta_t_callback(const std_msgs::Float32& msg){
	mhe_delta_t=msg.data;
}

void reference_position_callback(const geometry_msgs::Vector3& msg){
	reference_position=msg;
}

void calculated_force_callback(const geometry_msgs::Vector3& msg){
	calculated_force=msg;
}

void non_bias_external_force_callback(const geometry_msgs::Vector3& msg){
	non_bias_external_force=msg;
}

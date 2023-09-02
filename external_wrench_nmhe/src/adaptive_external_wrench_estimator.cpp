#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include "DoubleLinkedList.h"
#include <casadi/casadi.hpp>

using namespace casadi;

std_msgs::Float32MultiArray mhe_control_input;
std_msgs::Float32MultiArray mhe_measurement;

geometry_msgs::Vector3 external_force;
geometry_msgs::Vector3 external_torque;

int N_MHE=5;
int n_states=18;
int n_controls=6;
int n_outputs=12;
double Jxx=0.005; double Jyy=0.005; double Jzz=0.01;
double mass=2.42; double g=9.81; double PI=3.141592;

DM MHE_control_inputs = DM::zeros(n_controls,N_MHE);
DM MHE_measurements = DM::zeros(n_outputs,N_MHE+1);

MX F(const MX& x, const MX& u);
MX H(const MX& x);

Slice all_elem;
Opti opti;
MX obj=MX::zeros(1,1);
MX MHE_X=opti.variable(n_states,N_MHE+1);
//MX MHE_W=opti.variable(n_states,N_MHE);
//MX MHE_V=opti.variable(n_outputs,N_MHE+1);
MX MHE_Y=opti.parameter(n_outputs,N_MHE+1);
MX MHE_U=opti.parameter(n_controls,N_MHE);
MX MHE_init_X_tilde=opti.parameter(n_states,1);
MX MHE_sampling_time=opti.parameter(1,1);
MX MHE_mass=opti.parameter(1,1);
double output_stage_cost_weight=12.0;
double state_stage_cost_weight=1.0;
double arrival_cost_weight=1.0;

auto MHE_x_pos = MHE_X(0,all_elem);
auto MHE_y_pos = MHE_X(1,all_elem);
auto MHE_z_pos = MHE_X(2,all_elem);
auto MHE_x_vel = MHE_X(3,all_elem);
auto MHE_y_vel = MHE_X(4,all_elem);
auto MHE_z_vel = MHE_X(5,all_elem);
auto MHE_roll = MHE_X(6,all_elem);
auto MHE_pitch = MHE_X(7,all_elem);
auto MHE_yaw = MHE_X(8,all_elem);
auto MHE_omega_x = MHE_X(9,all_elem);
auto MHE_omega_y = MHE_X(10,all_elem);
auto MHE_omega_z = MHE_X(11,all_elem);
auto MHE_F_ex = MHE_X(12,all_elem);
auto MHE_F_ey = MHE_X(13,all_elem);
auto MHE_F_ez = MHE_X(14,all_elem);
auto MHE_tau_ex = MHE_X(15,all_elem);
auto MHE_tau_ey = MHE_X(16,all_elem);
auto MHE_tau_ez = MHE_X(17,all_elem);

DM MHE_X_star=DM::zeros(n_states,N_MHE+1);
//DM MHE_W_tilde=DM::zeros(n_states,N_MHE);
//DM MHE_V_tilde=DM::zeros(n_outputs,N_MHE+1);

Dict solver_option;

double sampling_time;

void mhe_get_measurements_control_inputs();
void mhe_nlp_setting();
void mhe_external_wrench_estimation();
void control_input_force_Callback(const geometry_msgs::Vector3& msg);
void control_input_torque_Callback(const geometry_msgs::Vector3& msg);
void measurement_position_Callback(const geometry_msgs::Vector3& msg);
void measurement_velocity_Callback(const geometry_msgs::Vector3& msg);
void measurement_attitude_Callback(const geometry_msgs::Vector3& msg);
void measurement_omega_Callback(const geometry_msgs::Vector3& msg);
void mass_Callback(const std_msgs::Float32& msg);

DoubleLinkedList measurement_x;
DoubleLinkedList measurement_y;
DoubleLinkedList measurement_z;
DoubleLinkedList measurement_x_dot;
DoubleLinkedList measurement_y_dot;
DoubleLinkedList measurement_z_dot;
DoubleLinkedList measurement_roll;
DoubleLinkedList measurement_pitch;
DoubleLinkedList measurement_yaw;
DoubleLinkedList measurement_omega_x;
DoubleLinkedList measurement_omega_y;
DoubleLinkedList measurement_omega_z;
DoubleLinkedList control_input_F_x;
DoubleLinkedList control_input_F_y;
DoubleLinkedList control_input_F_z;
DoubleLinkedList control_input_tau_x;
DoubleLinkedList control_input_tau_y;
DoubleLinkedList control_input_tau_z;

std_msgs::Float32 mhe_delta_t;
geometry_msgs::Vector3 force_d;
geometry_msgs::Vector3 torque_d;
geometry_msgs::Vector3 pos;
geometry_msgs::Vector3 vel;
geometry_msgs::Vector3 attitude;
geometry_msgs::Vector3 omega;

auto end=std::chrono::high_resolution_clock::now();
auto start=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> delta_t;

int main(int argc, char **argv){

	ros::init(argc, argv, "external_wrench_estimator");

	ros::NodeHandle nh;

	ros::Subscriber control_input_force_sub=nh.subscribe("/force_d",1,control_input_force_Callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber control_input_torque_sub=nh.subscribe("/torque_d",1,control_input_torque_Callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber measurement_position_sub=nh.subscribe("/pos",1,measurement_position_Callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber measurement_velocity_sub=nh.subscribe("/lin_vel",1,measurement_velocity_Callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber measurement_attitude_sub=nh.subscribe("/angle",1,measurement_attitude_Callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber measurement_omega_sub=nh.subscribe("/angular_velocity",1,measurement_omega_Callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber mass_sub=nh.subscribe("/mass",1,mass_Callback,ros::TransportHints().tcpNoDelay());

	ros::Publisher external_force_pub=nh.advertise<geometry_msgs::Vector3>("adaptive_external_force",1);
	ros::Publisher external_torque_pub=nh.advertise<geometry_msgs::Vector3>("adaptive_external_torque",1);
	ros::Publisher mhe_delta_t_pub=nh.advertise<std_msgs::Float32>("adaptive_mhe_delta_t",1);	

//	N_MHE=nh.param<int>("MHE_window_length",10);
	
	mhe_nlp_setting();

	while(ros::ok()){
		
		end=std::chrono::high_resolution_clock::now();
		delta_t=end-start;
		mhe_delta_t.data=delta_t.count();
//		sampling_time=delta_t.count();
		start=std::chrono::high_resolution_clock::now();

		mhe_get_measurements_control_inputs();
		mhe_external_wrench_estimation();
		
		external_force_pub.publish(external_force);
		external_torque_pub.publish(external_torque);
		mhe_delta_t_pub.publish(mhe_delta_t);

		ros::spinOnce();
	}
	
}

MX F(const MX& x, const MX& u){
	MX dx=MX::zeros(n_states,1);
	dx(0)=x(3);
	dx(1)=x(4);
	dx(2)=x(5);
	dx(3)=(x(12) - u(1) * (cos(x(6)) * sin(x(8)) - cos(x(8)) * sin(x(6)) * sin(x(7))) + u(2) * (sin(x(6)) * sin(x(8)) + cos(x(6)) * cos(x(8)) * sin(x(7))) + u(0) * cos(x(8)) * cos(x(7))) / MHE_mass;
	dx(4)=(x(13) + u(1) * (cos(x(6)) * cos(x(8)) + sin(x(6)) * sin(x(8)) * sin(x(7))) - u(2) * (cos(x(8)) * sin(x(6)) - cos(x(6)) * sin(x(8)) * sin(x(7))) + u(0) * cos(x(7)) * sin(x(8))) / MHE_mass;
	dx(5)=g + (x(14) - u(0) * sin(x(7)) + u(2) * cos(x(6)) * cos(x(7)) + u(1) * cos(x(7)) * sin(x(6))) / MHE_mass;
	dx(6)=x(9) + sin(x(6)) * tan(x(7)) * x(10) + cos(x(6)) * tan(x(7)) * x(11);
	dx(7)=cos(x(6)) * x(10) - sin(x(6)) * x(11);
	dx(8)=sin(x(6)) / cos(x(7)) * x(10) + cos(x(6)) / cos(x(7)) * x(11);
	dx(9)=(u(3) + x(15) - (Jzz - Jyy)*x(10)*x(11)) / Jxx;
   	dx(10)=(u(4) + x(16) - (Jxx - Jzz)*x(9)*x(11)) / Jyy;
      	dx(11)=(u(5) + x(17) - (Jyy - Jxx)*x(9)*x(10)) / Jzz;
	dx(12)=0; dx(13)=0; dx(14)=0;
	dx(15)=0; dx(16)=0; dx(17)=0;
	return x+dx*MHE_sampling_time;
}

MX H(const MX& x){
	return x(Slice(0,n_outputs));
}

void mhe_get_measurements_control_inputs(){
	if(measurement_x.length()==(N_MHE+1)) measurement_x.deleteHead();
	if(measurement_y.length()==(N_MHE+1)) measurement_y.deleteHead();
	if(measurement_z.length()==(N_MHE+1)) measurement_z.deleteHead();
	if(measurement_x_dot.length()==(N_MHE+1)) measurement_x_dot.deleteHead();
	if(measurement_y_dot.length()==(N_MHE+1)) measurement_y_dot.deleteHead();
	if(measurement_z_dot.length()==(N_MHE+1)) measurement_z_dot.deleteHead();
	if(measurement_roll.length()==(N_MHE+1)) measurement_roll.deleteHead();
	if(measurement_pitch.length()==(N_MHE+1)) measurement_pitch.deleteHead();
	if(measurement_yaw.length()==(N_MHE+1)) measurement_yaw.deleteHead();
	if(measurement_omega_x.length()==(N_MHE+1)) measurement_omega_x.deleteHead();
	if(measurement_omega_y.length()==(N_MHE+1)) measurement_omega_y.deleteHead();
	if(measurement_omega_z.length()==(N_MHE+1)) measurement_omega_z.deleteHead();
	if(control_input_F_x.length()==N_MHE+1) control_input_F_x.deleteHead();
	if(control_input_F_y.length()==N_MHE+1) control_input_F_y.deleteHead();
	if(control_input_F_z.length()==N_MHE+1) control_input_F_z.deleteHead();
	if(control_input_tau_x.length()==N_MHE+1) control_input_tau_x.deleteHead();
	if(control_input_tau_y.length()==N_MHE+1) control_input_tau_y.deleteHead();
	if(control_input_tau_z.length()==N_MHE+1) control_input_tau_z.deleteHead();

	measurement_x.insertTail(pos.x);
	measurement_y.insertTail(pos.y);
	measurement_z.insertTail(pos.z);
	measurement_x_dot.insertTail(vel.x);
	measurement_y_dot.insertTail(vel.y);
	measurement_z_dot.insertTail(vel.z);
	measurement_roll.insertTail(attitude.x);
	measurement_pitch.insertTail(attitude.y);
	measurement_yaw.insertTail(attitude.z);
	measurement_omega_x.insertTail(omega.x);
	measurement_omega_y.insertTail(omega.y);
	measurement_omega_z.insertTail(omega.z);
	control_input_F_x.insertTail(force_d.x);
	control_input_F_y.insertTail(force_d.y);
	control_input_F_z.insertTail(force_d.z);
	control_input_tau_x.insertTail(0.0);
	control_input_tau_y.insertTail(0.0);
	control_input_tau_z.insertTail(torque_d.z);

	for(int i=0; i<N_MHE+1; i++){
		MHE_measurements(0,i)=measurement_x.getData(i);
		MHE_measurements(1,i)=measurement_y.getData(i);
		MHE_measurements(2,i)=measurement_z.getData(i);
		MHE_measurements(3,i)=measurement_x_dot.getData(i);
		MHE_measurements(4,i)=measurement_y_dot.getData(i);
		MHE_measurements(5,i)=measurement_z_dot.getData(i);
		MHE_measurements(6,i)=measurement_roll.getData(i);
		MHE_measurements(7,i)=measurement_pitch.getData(i);
		MHE_measurements(8,i)=measurement_yaw.getData(i);
		MHE_measurements(9,i)=measurement_omega_x.getData(i);
		MHE_measurements(10,i)=measurement_omega_y.getData(i);
		MHE_measurements(11,i)=measurement_omega_z.getData(i);

		if(i<N_MHE){
			MHE_control_inputs(0,i)=control_input_F_x.getData(i);
			MHE_control_inputs(1,i)=control_input_F_y.getData(i);
			MHE_control_inputs(2,i)=control_input_F_z.getData(i);
			MHE_control_inputs(3,i)=control_input_tau_x.getData(i);
			MHE_control_inputs(4,i)=control_input_tau_y.getData(i);
			MHE_control_inputs(5,i)=control_input_tau_z.getData(i);
		}
	}
}

void mhe_nlp_setting(){
	for(int k=0; k<N_MHE+1; k++){
		MX st=MHE_X(all_elem,k);
		MX y_tilde=MHE_Y(all_elem,k);
		MX H_x=H(st);
		obj+=mtimes((y_tilde-H_x).T(),(y_tilde-H_x))*output_stage_cost_weight;
		
		if(k<N_MHE){
			MX con=MHE_U(all_elem,k);
			MX next_st=MHE_X(all_elem,k+1);
			MX F_xu=F(st,con);
			obj+=mtimes((next_st-F_xu).T(),(next_st-F_xu))*state_stage_cost_weight;
		}
	}
	MX init_st=MHE_X(all_elem,0);
	obj+=mtimes((init_st-MHE_init_X_tilde).T(),(init_st-MHE_init_X_tilde))*arrival_cost_weight;

	opti.minimize(obj);

	for(int k=0;k<N_MHE;k++){
		MX st=MHE_X(all_elem,k);
		MX con=MHE_U(all_elem,k);
		MX st_next=F(st,con);//+MHE_W(all_elem,k);
		opti.subject_to(MHE_X(all_elem,k+1)==st_next);	
	}
/*
	for(int k=0;k<N_MHE+1;k++){
		MX st=MHE_X(all_elem,k);
		MX output_vec=H(st);//+MHE_V(all_elem,k);
		opti.subject_to(MHE_Y(all_elem,k)==output_vec);
	}
*/
//	opti.subject_to(-5<=MHE_x_pos<=5);
//	opti.subject_to(-5<=MHE_y_pos<=5);
//	opti.subject_to(-3<=MHE_z_pos<=0.5);
	opti.subject_to(-3<=MHE_x_vel<=3);
	opti.subject_to(-3<=MHE_y_vel<=3);
	opti.subject_to(-5<=MHE_z_vel<=5);
	opti.subject_to(-0.7<=MHE_roll<=0.7);
	opti.subject_to(-0.7<=MHE_pitch<=0.7);
//	opti.subject_to(-PI<=MHE_yaw<=PI);
	opti.subject_to(-2<=MHE_omega_x<=2);
	opti.subject_to(-2<=MHE_omega_y<=2);
	opti.subject_to(-2<=MHE_omega_z<=2);
	opti.subject_to(-5<=MHE_F_ex<=5);
	opti.subject_to(-5<=MHE_F_ey<=5);
//	opti.subject_to(-5<=MHE_F_ez<=5);
	opti.subject_to(-3<=MHE_tau_ex<=3);
	opti.subject_to(-3<=MHE_tau_ey<=3);
	opti.subject_to(-3<=MHE_tau_ez<=3);	

	opti.set_initial(MHE_x_pos,0);
	opti.set_initial(MHE_y_pos,0);
	opti.set_initial(MHE_z_pos,0);
	opti.set_initial(MHE_x_vel,0);
	opti.set_initial(MHE_y_vel,0);
	opti.set_initial(MHE_z_vel,0);
	opti.set_initial(MHE_roll,0);
	opti.set_initial(MHE_pitch,0);
	opti.set_initial(MHE_yaw,PI/4.0);
	opti.set_initial(MHE_omega_x,0);
	opti.set_initial(MHE_omega_y,0);
	opti.set_initial(MHE_omega_z,0);
	opti.set_initial(MHE_F_ex,0);
	opti.set_initial(MHE_F_ey,0);
	opti.set_initial(MHE_F_ez,-mass*g);
	opti.set_initial(MHE_tau_ex,0);
	opti.set_initial(MHE_tau_ey,0);
	opti.set_initial(MHE_tau_ez,0);

//	opti.set_initial(MHE_W,MHE_W_tilde);
//	opti.set_initial(MHE_V,MHE_V_tilde);

	solver_option["ipopt.max_iter"]=100;
	solver_option["print_time"]=0;
	solver_option["ipopt.print_level"]=0;
	solver_option["ipopt.tol"]=1e-0;
	solver_option["ipopt.acceptable_tol"]=1e-0;
	solver_option["ipopt.acceptable_obj_change_tol"]=1e-6;

	opti.solver("ipopt",solver_option);	

}

void mhe_external_wrench_estimation(){

	opti.set_value(MHE_init_X_tilde,MHE_X_star(all_elem,1));	
	opti.set_value(MHE_Y,MHE_measurements);
	opti.set_value(MHE_U,MHE_control_inputs);
	opti.set_value(MHE_sampling_time,delta_t.count());
	opti.set_value(MHE_mass,mass);
	
	OptiSol sol=opti.solve();
	MHE_X_star=sol.value(MHE_X);
//	MHE_W_tilde=sol.value(MHE_W);
//	MHE_V_tilde=sol.value(MHE_V);

//	std::cout << opti.debug().value(MHE_mass) << std::endl;
	external_force.x=(double)MHE_X_star(12,N_MHE);
	external_force.y=(double)MHE_X_star(13,N_MHE);
	external_force.z=(double)MHE_X_star(14,N_MHE);
	external_torque.x=(double)MHE_X_star(15,N_MHE);
	external_torque.y=(double)MHE_X_star(16,N_MHE);
	external_torque.z=(double)MHE_X_star(17,N_MHE);

	opti.set_initial(MHE_x_pos,horzcat(MHE_X_star(0,Slice(1,N_MHE+1,1)),MHE_X_star(0,N_MHE)));
	opti.set_initial(MHE_y_pos,horzcat(MHE_X_star(1,Slice(1,N_MHE+1,1)),MHE_X_star(1,N_MHE)));
	opti.set_initial(MHE_z_pos,horzcat(MHE_X_star(2,Slice(1,N_MHE+1,1)),MHE_X_star(2,N_MHE)));
	opti.set_initial(MHE_x_vel,horzcat(MHE_X_star(3,Slice(1,N_MHE+1,1)),MHE_X_star(3,N_MHE)));
	opti.set_initial(MHE_y_vel,horzcat(MHE_X_star(4,Slice(1,N_MHE+1,1)),MHE_X_star(4,N_MHE)));
	opti.set_initial(MHE_z_vel,horzcat(MHE_X_star(5,Slice(1,N_MHE+1,1)),MHE_X_star(5,N_MHE)));
	opti.set_initial(MHE_roll,horzcat(MHE_X_star(6,Slice(1,N_MHE+1,1)),MHE_X_star(6,N_MHE)));
	opti.set_initial(MHE_pitch,horzcat(MHE_X_star(7,Slice(1,N_MHE+1,1)),MHE_X_star(7,N_MHE)));
	opti.set_initial(MHE_yaw,horzcat(MHE_X_star(8,Slice(1,N_MHE+1,1)),MHE_X_star(8,N_MHE)));
	opti.set_initial(MHE_omega_x,horzcat(MHE_X_star(9,Slice(1,N_MHE+1,1)),MHE_X_star(9,N_MHE)));
	opti.set_initial(MHE_omega_y,horzcat(MHE_X_star(10,Slice(1,N_MHE+1,1)),MHE_X_star(10,N_MHE)));
	opti.set_initial(MHE_omega_z,horzcat(MHE_X_star(11,Slice(1,N_MHE+1,1)),MHE_X_star(11,N_MHE)));
	opti.set_initial(MHE_F_ex,horzcat(MHE_X_star(12,Slice(1,N_MHE+1,1)),MHE_X_star(12,N_MHE)));
	opti.set_initial(MHE_F_ey,horzcat(MHE_X_star(13,Slice(1,N_MHE+1,1)),MHE_X_star(13,N_MHE)));
	opti.set_initial(MHE_F_ez,horzcat(MHE_X_star(14,Slice(1,N_MHE+1,1)),MHE_X_star(14,N_MHE)));
	opti.set_initial(MHE_tau_ex,horzcat(MHE_X_star(15,Slice(1,N_MHE+1,1)),MHE_X_star(15,N_MHE)));
	opti.set_initial(MHE_tau_ey,horzcat(MHE_X_star(16,Slice(1,N_MHE+1,1)),MHE_X_star(16,N_MHE)));
	opti.set_initial(MHE_tau_ez,horzcat(MHE_X_star(17,Slice(1,N_MHE+1,1)),MHE_X_star(17,N_MHE)));

//	opti.set_initial(MHE_W,MHE_W_tilde);
//	opti.set_initial(MHE_V,MHE_V_tilde);
}

void control_input_force_Callback(const geometry_msgs::Vector3& msg){
	force_d=msg;
}

void control_input_torque_Callback(const geometry_msgs::Vector3& msg){
	torque_d=msg;
}

void measurement_position_Callback(const geometry_msgs::Vector3& msg){
	pos=msg;
}

void measurement_velocity_Callback(const geometry_msgs::Vector3& msg){
	vel=msg;
}

void measurement_attitude_Callback(const geometry_msgs::Vector3& msg){
	attitude=msg;
}

void measurement_omega_Callback(const geometry_msgs::Vector3& msg){
	omega=msg;
}

void mass_Callback(const std_msgs::Float32& msg){
	mass=msg.data;
}

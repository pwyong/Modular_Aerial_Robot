//2022.05.16 Coaxial-Octorotor version
//2022.06.23 Ground Station Application
//2022.08.XX DOB (Disturbance Observer) Application
//2022.09.05 ESC (Extremum Seeking Control) Application
//2022.09.21 Controller mode selection Application

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>
#include <vector>
#include <cmath>
#include <cstdio>
#include <chrono>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Imu.h>
//#include "FAC_MAV/FAC_MAV_ctrler.h"

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "FAC_MAV/ArmService.h" //ASDF
#include "FAC_MAV/KillService.h" //ASDF
#include "FAC_MAV/PosCtrlService.h" //ASDF
#include "FAC_MAV/HoverService.h" //ASDF
#include "FAC_MAV/FAC_HoverService.h" //ASDF

#include "nav_msgs/Odometry.h"

double freq=200;//controller loop frequency
double pwm_freq=417.3;//pwm signal frequency

std::chrono::duration<double> delta_t;
int16_t Sbus[9];
int16_t PWM_d;
int16_t loop_time;
std_msgs::Int16MultiArray PWMs_cmd;
std_msgs::Int32MultiArray PWMs_val;
std_msgs::Float32MultiArray Force;

sensor_msgs::JointState rac_servo_value;
sensor_msgs::Imu imu;
geometry_msgs::Quaternion imu_quaternion;
geometry_msgs::Vector3 imu_rpy;
geometry_msgs::Vector3 imu_ang_vel;
geometry_msgs::Vector3 imu_lin_acc;
geometry_msgs::Vector3 angle_d;
geometry_msgs::Vector3 pos;
geometry_msgs::Vector3 t265_lin_vel;
geometry_msgs::Vector3 t265_ang_vel;
geometry_msgs::Vector3 lin_vel;
geometry_msgs::Vector3 prev_lin_vel;
geometry_msgs::Vector3 ang_vel;
geometry_msgs::Quaternion t265_quat;
geometry_msgs::Quaternion rot;
geometry_msgs::Quaternion desired_value;
geometry_msgs::Vector3 desired_pos;
geometry_msgs::Vector3 F_total;
geometry_msgs::Vector3 torque_d;
geometry_msgs::Vector3 force_d;
std_msgs::Float32MultiArray force_cmd;
geometry_msgs::Vector3 desired_lin_vel;
geometry_msgs::Vector3 t265_att;
geometry_msgs::Vector3 filtered_angular_rate;
geometry_msgs::Vector3 lin_acl;
std_msgs::Float32 altitude_d;
std_msgs::Float32 battery_voltage_msg;
std_msgs::Float32 battery_real_voltage;
std_msgs::Float32 dt;
bool servo_sw=false;
double theta1_command, theta2_command;
bool start_flag=false;
bool tilting_flag=false;

//Mode selection flag
bool attitude_mode = false;
bool velocity_mode = false;
bool position_mode = false;
bool kill_mode = true;
bool altitude_mode = false;
bool tilt_mode = false;
bool ESC_control = false;

//State
bool hovering = false;
bool loading = false;

//Thruster_cmd
double F1 = 0;//desired propeller 1 force
double F2 = 0;//desired propeller 2 force
double F3 = 0;//desired propeller 3 force
double F4 = 0;//desired propeller 4 force
double F5 = 0;//desired propeller 5 force
double F6 = 0;//desired propeller 6 force
double F7 = 0;//desired propeller 7 force
double F8 = 0;//desired propeller 8 force

//Global : XYZ  Body : xyz
double e_r_i = 0;//roll error integration
double e_p_i = 0;//pitch error integration
double e_y_i = 0;//yaw error integration
double e_X_i = 0;//X position error integration
double e_Y_i = 0;//Y position error integration
double e_Z_i = 0;//Z position error integration
double e_X_dot_i = 0;//X velocity error integration
double e_Y_dot_i = 0;//Y velocity error integration
double e_Z_dot_i = 0;//Z velocity error integration

double tau_r_d = 0;//roll  desired torque (N.m)
double tau_p_d = 0;//pitch desired torque(N.m)
double tau_y_d = 0;//yaw desired torque (N.m)

double Thrust_d = 0;//altitiude desired thrust(N)
//ud_cmd

double r_d = 0;//desired roll angle
double p_d = 0;//desired pitch angle
double y_d = 0;//desired yaw angle
double y_d_tangent = 0;//yaw increment tangent
double T_d = 0;//desired thrust

//Desired Global position
double X_d = 0;//desired X position
double Y_d = 0;//desired Y position
double Z_d = 0;//desired altitude
double X_d_base = 0;//initial desired X position
double Y_d_base = 0;//initial desired Y position
double Z_d_base = 0;//initial desired Z position

//Global Desired Global velocity
double X_dot_d = 0;
double Y_dot_d = 0;
double Z_dot_d = 0;

//Global desired acceleration
double X_ddot_d = 0;
double Y_ddot_d = 0;
double Z_ddot_d = 0;

double alpha = 0;
double beta = 0;

//Body desired force
double F_xd = 0;
double F_yd = 0;
double F_zd = 0;

//Yaw safety
double yaw_prev = 0;
double yaw_now = 0;
double base_yaw = 0;
int yaw_rotate_count = 0;
//--------------------------------------------------------

//General dimensions

static double r_arm = 0.109;// m // diagonal length between thruster : 218mm;
static double l_servo = 0.015;
static double mass = 2.365;//(Kg)


//Propeller constants(DJI E800(3510 motors + 620S ESCs))
static double b_over_k_ratio = 0.01;//F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
//--------------------------------------------------------

//General parameters======================================

static double pi = 3.141592;//(rad)
static double g = 9.80665;//(m/s^2)

static double rp_limit = 0.3;//(rad)
static double y_vel_limit = 0.01;//(rad/s)
static double y_d_tangent_deadzone = (double)0.05 * y_vel_limit;//(rad/s)
static double T_limit = 17;//(N)
static double altitude_limit = 1;//(m)
static double XY_limit = 0.5;
static double XYZ_dot_limit=1;
static double XYZ_ddot_limit=2;
static double alpha_beta_limit=1;
static double hardware_servo_limit=0.3;
static double servo_command_limit = 0.2;
static double tau_y_limit = 0.3;

double x_c_hat=0.0;
double y_c_hat=0.0;
double z_c_hat=0.0;
//--------------------------------------------------------

//Control gains===========================================

//integratior(PID) limitation
double integ_limit=10;
double z_integ_limit=100;
double pos_integ_limit=10;
double vel_integ_limit=10;

//Roll, Pitch PID gains
double Pa=3.5;
double Ia=0.4;
double Da=0.5;


//Yaw PID gains
double Py=2.0;
double Dy=0.1;

//Z Velocity PID gains
double Pz=16.0;
double Iz=5.0;
double Dz=15.0;

//XY Velocity PID gains
double Pv=5.0;
double Iv=0.1;
double Dv=5.0;

//Position PID gains
double Pp=3.0;
double Ip=0.1;
double Dp=5.0;

//Conventional Flight Mode Control Gains
double conv_Pa, conv_Ia, conv_Da;
double conv_Py, conv_Dy;
double conv_Pz, conv_Iz, conv_Dz;
double conv_Pv, conv_Iv, conv_Dv;
double conv_Pp, conv_Ip, conv_Dp;

//Tilt Flight Mode Control Gains
double tilt_Pa, tilt_Ia, tilt_Da;
double tilt_Py, tilt_Dy;
double tilt_Pz, tilt_Iz, tilt_Dz;
double tilt_Pv, tilt_Iv, tilt_Dv;
double tilt_Pp, tilt_Ip, tilt_Dp;
//--------------------------------------------------------

//Servo angle=============================================
double theta1=0,theta2=0;
//--------------------------------------------------------

//Voltage=================================================
double voltage=16.0;
double voltage_old=16.0;
//--------------------------------------------------------


//-DOB----------------------------------------------------
geometry_msgs::Vector3 dhat;
double fq_cutoff=1.0;//Q filter Cut-off frequency

// Nominal MoI
double J_x = 0.01;
double J_y = 0.01;
double J_z = 0.1;

//Roll DOB
double x_r1=0;
double x_r2=0;
double x_r3=0;
double x_dot_r1=0;
double x_dot_r2=0;
double x_dot_r3=0;

double y_r1=0;
double y_r2=0;
double y_r3=0;
double y_dot_r1=0;
double y_dot_r2=0;
double y_dot_r3=0;

double dhat_r = 0;
double tautilde_r_d=0;

//Pitch DOB
double x_p1=0;
double x_p2=0;
double x_p3=0;
double x_dot_p1=0;
double x_dot_p2=0;
double x_dot_p3=0;

double y_p1=0;
double y_p2=0;
double y_p3=0;
double y_dot_p1=0;
double y_dot_p2=0;
double y_dot_p3=0;

double dhat_p=0;
double tautilde_p_d=0;

//Yaw DOB
double x_y1=0;
double x_y2=0;
double x_y3=0;
double x_dot_y1=0;
double x_dot_y2=0;
double x_dot_y3=0;

double y_y1=0;
double y_y2=0;
double y_y3=0;
double y_dot_y1=0;
double y_dot_y2=0;
double y_dot_y3=0;

double tautilde_y_d=0;
//--------------------------------------------------------


//Publisher Group--------------------------------------
ros::Publisher PWMs;
ros::Publisher goal_dynamixel_position_;
ros::Publisher euler;
ros::Publisher desired_angle;
ros::Publisher Forces;
ros::Publisher desired_torque;
ros::Publisher linear_velocity;
ros::Publisher angular_velocity;
ros::Publisher PWM_generator;
ros::Publisher desired_position;
ros::Publisher position;
ros::Publisher kalman_angular_vel;
ros::Publisher kalman_angular_accel;
ros::Publisher desired_force;
ros::Publisher battery_voltage;
ros::Publisher force_command;
ros::Publisher delta_time;
ros::Publisher desired_velocity;
ros::Publisher Center_of_Mass;
ros::Publisher angular_Acceleration;
ros::Publisher sine_wave_data;
ros::Publisher disturbance;
ros::Publisher linear_acceleration;
//----------------------------------------------------

//Control Matrix---------------------------------------
//Eigen::MatrixXd CM(4,8);
Eigen::MatrixXd CM(4,4);
//Eigen::Vector4d u;
Eigen::VectorXd u(4);
Eigen::VectorXd F_cmd(4);
Eigen::MatrixXd invCM(4,4);
//-----------------------------------------------------

//Linear_velocity--------------------------------------
Eigen::Vector3d cam_v;
Eigen::Matrix3d R_v;
Eigen::Vector3d v;
//-----------------------------------------------------

//Attitude--------------------------------------
Eigen::Vector3d cam_att;
//-----------------------------------------------------

//Timer------------------------------------------------
auto end  =std::chrono::high_resolution_clock::now();
auto start=std::chrono::high_resolution_clock::now();
//-----------------------------------------------------

//Extremum Seeking Control-----------------------------
geometry_msgs::Vector3 prev_angular_Vel;
geometry_msgs::Vector3 angular_Accel;
geometry_msgs::Vector3 CoM;
geometry_msgs::Vector3 sine_wave;

double MoI_x_hat = 0.01;
double MoI_y_hat = 0.01;
double G_XY = 0.5;
double G_Z = 0.5;

double bias_x_c = 0;
double bias_y_c = 0;
double bias_z_c = 0;
double x_c_limit = 0.04;
double y_c_limit = 0.04;
double z_c_limit = 0.1;

//Bandpass filter parameter
double Q_factor=10;
double pass_freq1=3.0;
double pass_freq2=5.0;

//Filter1
double x_11=0;
double x_12=0;
double x_dot_11=0;
double x_dot_12=0;
double y_11=0;

//Filter2
double x_21=0;
double x_22=0;
double x_dot_21=0;
double x_dot_22=0;
double y_21=0;

//Filter3
double x_31=0;
double x_32=0;
double x_dot_31=0;
double x_dot_32=0;
double y_31=0;

double vibration1=0;
double vibration2=0;
double time_count=0;
double Amp_XY=0.5;
double Amp_Z=1.0;

//gradient LPF - 3rd Butterworth
double xy_cutoff_freq = 1.5;
double x_grad_x_dot_1=0;
double x_grad_x_dot_2=0;
double x_grad_x_dot_3=0;
double x_grad_x_1=0;
double x_grad_x_2=0;
double x_grad_x_3=0;
double filtered_grad_x=0;

double x_grad_y_dot_1=0;
double x_grad_y_dot_2=0;
double x_grad_y_dot_3=0;
double x_grad_y_1=0;
double x_grad_y_2=0;
double x_grad_y_3=0;
double filtered_grad_y=0;

double z_cutoff_freq = 2.5;
double x_grad_z_dot_1=0;
double x_grad_z_dot_2=0;
double x_grad_z_dot_3=0;
double x_grad_z_1=0;
double x_grad_z_2=0;
double x_grad_z_3=0;
double filtered_grad_z=0;
//-----------------------------------------------------
//Accelerometer LPF------------------------------------
double x_ax_dot = 0;
double x_ay_dot = 0;
double x_ax = 0;
double x_ay = 0;
double accel_cutoff_freq = 5.0;
//-----------------------------------------------------

//Function------------------------------------------------
template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setCM(){
	//Co-rotating type
	CM <<          (l_servo+z_c_hat)*sin(theta1)+y_c_hat*cos(theta1),  (r_arm+y_c_hat)*cos(theta2)+b_over_k_ratio*sin(theta2),       (l_servo+z_c_hat)*sin(theta1)+y_c_hat*cos(theta1), -(r_arm-y_c_hat)*cos(theta2)+b_over_k_ratio*sin(theta2),
              (r_arm-x_c_hat)*cos(theta1)+b_over_k_ratio*sin(theta1),      (-l_servo+z_c_hat)*sin(theta2)-x_c_hat*cos(theta2), -(r_arm+x_c_hat)*cos(theta1)+b_over_k_ratio*sin(theta1),      (-l_servo+z_c_hat)*sin(theta2)-x_c_hat*cos(theta2),
              (r_arm-x_c_hat)*sin(theta1)-b_over_k_ratio*cos(theta1), -(r_arm+y_c_hat)*sin(theta2)+b_over_k_ratio*cos(theta2), -(r_arm+x_c_hat)*sin(theta1)-b_over_k_ratio*cos(theta1),  (r_arm-y_c_hat)*sin(theta2)+b_over_k_ratio*cos(theta2),
                                                        -cos(theta1),                                            -cos(theta2),                                            -cos(theta1),                                            -cos(theta2);
    	invCM = CM.inverse();
}

 

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des) {	
 	
	//Co-rotating coaxial
	//Conventional type
	F_cmd = invCM*u;
	if(Sbus[8]<=1500){
		theta1_command = 0.0;
        theta2_command = 0.0;
	}
	//Tilting type
	else {
		theta1_command = asin(F_yd/(F_cmd(0)+F_cmd(2)));
		theta2_command = asin(-F_xd/(F_cmd(1)+F_cmd(3)));
 		if(fabs(theta1_command)>hardware_servo_limit) theta1_command = (theta1_command/fabs(theta1_command))*hardware_servo_limit;
		if(fabs(theta2_command)>hardware_servo_limit) theta2_command = (theta2_command/fabs(theta2_command))*hardware_servo_limit;
	}
	F1 = F_cmd(0);
	F2 = F_cmd(1);
	F3 = F_cmd(2);
	F4 = F_cmd(3);

	pwm_Command(Force_to_PWM(F1),Force_to_PWM(F2), Force_to_PWM(F3), Force_to_PWM(F4), Force_to_PWM(F1), Force_to_PWM(F2), Force_to_PWM(F3), Force_to_PWM(F4));
	
	// ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
	// ROS_INFO("%f 1:%d, 2:%d, 3:%d, 4:%d",z_d,PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
}

 

double Force_to_PWM(double F) {
	double pwm;
	double A = -9.8*pow(10.0,-8.0)*pow(voltage,2.0)+3.23*pow(10.0,-6.0)*voltage-1.8*pow(10.0,-5.0);
	double B = 0.000243*pow(voltage,2.0)-0.00663*voltage+0.03723;
	double C = -0.11063*pow(voltage,2.0)+2.332691*voltage-10.885;
	double param1 = -B/(2.0*A);
	double param2 = 1.0/A;
	double param3 = (pow(B,2.0)-4*A*C)/(4*pow(A,2.0));
	//Force=A*pwm^2+B*pwm+C
	if(param2*F+param3>0){
		pwm = param1 + sqrt(param2 * F + param3);
	}
	else pwm = param1;
	if (pwm > 1900)	pwm = 1900;
	if(pwm < 1100) pwm = 1100;
	if(Sbus[5]>1500){
		if(Z_d_base<=0){
			if(Z_d>Z_d_base && !start_flag) {
				pwm=param1;
			}
			else if(Z_d<Z_d_base) start_flag=true;
		}
		else pwm=param1;
	}
	return pwm;
}

void jointstateCallback(const sensor_msgs::JointState& msg){
    	rac_servo_value=msg;
	theta1=msg.position[0];
	theta2=-msg.position[1];
    	//ROS_INFO("theta1:%lf   theta2:%lf",theta1, theta2);
}

ros::Time imuTimer;

void imu_Callback(const sensor_msgs::Imu& msg){
    	imu=msg;
    
    	// TP attitude - Quaternion representation
    	imu_quaternion=msg.orientation;
    	imu_ang_vel=msg.angular_velocity;
    	// ROS_INFO("R:%lf\tP:%lf\tY:%lf",imu_ang_vel.x,imu_ang_vel.y,imu_ang_vel.z);
    	imu_lin_acc=msg.linear_acceleration;

    	tf::Quaternion quat;
    	tf::quaternionMsgToTF(imu_quaternion,quat);

    	// TP attitude - Euler representation
    	tf::Matrix3x3(quat).getRPY(imu_rpy.x,imu_rpy.y,imu_rpy.z);
	base_yaw = cam_att(2);
    	if(base_yaw - yaw_prev < -pi) yaw_rotate_count++;
	else if(base_yaw - yaw_prev > pi) yaw_rotate_count--;
	yaw_now = base_yaw+2*pi*yaw_rotate_count;
	//ROS_INFO("now : %lf / prev : %lf / count : %d",yaw_now, yaw_prev, yaw_rotate_count);
	imu_rpy.z = yaw_now;
	yaw_prev = base_yaw;
	// ROS_INFO("imuCallback time : %f",(((double)ros::Time::now().sec-(double)imuTimer.sec)+((double)ros::Time::now().nsec-(double)imuTimer.nsec)/1000000000.));
	//imuTimer = ros::Time::now();
}

void filterCallback(const sensor_msgs::Imu& msg){
	filtered_angular_rate=msg.angular_velocity;
}

sensor_msgs::JointState servo_msg_create(double rr, double rp){
	sensor_msgs::JointState servo_msg;

	servo_msg.header.stamp=ros::Time::now();

	servo_msg.name.resize(2);
	servo_msg.name[0]="id_1";
	servo_msg.name[1]="id_2";

	servo_msg.position.resize(2);
	servo_msg.position[0]=rr;
	servo_msg.position[1]=rp;
	//ROS_INFO("rr: %lf, rp: %lf",rr,rp);
	return servo_msg;
}

void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array){
	for(int i=0;i<10;i++){
		Sbus[i]=map<int16_t>(array->data[i], 352, 1696, 1000, 2000);
	}
	
	if(Sbus[4]<1500) kill_mode=true;
	else kill_mode=false;
	
	if(Sbus[5]>1500) altitude_mode=true;
	else altitude_mode=false;

	if(Sbus[6]<1300){
		attitude_mode=true;
		velocity_mode=false;
		position_mode=false;
	}
	else if(Sbus[6]<1700){
		attitude_mode=false;
		velocity_mode=true;
		position_mode=false;
	}
	else{
		attitude_mode=false;
		velocity_mode=false;
		position_mode=true;
	}

	if(Sbus[8]>1500) tilt_mode=true;
	else tilt_mode=false;
	
	if(Sbus[9]>1500) ESC_control=true;
	else ESC_control=false;
}


void batteryCallback(const std_msgs::Int16& msg){
	int16_t value=msg.data;
	voltage=value*3.3/(double)4096/(7440./(30000.+7440.));
	double kv=0.08;
	voltage=kv*voltage+(1-kv)*voltage_old;
	voltage_old=voltage;
	if(voltage>16.8) voltage=16.8;
	if(voltage<13.0) voltage=13.0;
	battery_voltage_msg.data=voltage;
}

ros::Time posTimer;
void posCallback(const geometry_msgs::Vector3& msg){
	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=msg.z;
}

void rotCallback(const geometry_msgs::Quaternion& msg){
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;
	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);	
}

void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	t265_lin_vel=msg->twist.twist.linear;
	t265_ang_vel=msg->twist.twist.angular;
	t265_quat=msg->pose.pose.orientation;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(cam_att(0),cam_att(1),cam_att(2));
	cam_v << t265_lin_vel.x, t265_lin_vel.y, t265_lin_vel.z;
	R_v << cos(pi/2.), -sin(pi/2.),  0.,
 	       sin(pi/2.),  cos(pi/2.),  0.,
	                0.,           0.,  1.;

	v = R_v*cam_v;

	double global_X_dot = v(2)*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-v(1)*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.z)*cos(imu_rpy.y);
	double global_Y_dot = v(1)*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))-v(2)*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.y)*sin(imu_rpy.z);
	double global_Z_dot = -v(0)*sin(imu_rpy.y)+v(2)*cos(imu_rpy.x)*cos(imu_rpy.y)+v(1)*cos(imu_rpy.y)*sin(imu_rpy.x);

	lin_vel.x=v(0);//global_X_dot;
	lin_vel.y=v(1);//global_Y_dot;
	lin_vel.z=v(2);//global_Z_dot;
	//ROS_INFO("Attitude - [r: %f  p: %f  y:%f]",cam_att(0),cam_att(1),cam_att(2));
	//ROS_INFO("Linear_velocity - [x: %f  y: %f  z:%f]",v(0),v(1),v(2));
	//ROS_INFO("Angular_velocity - [x: %f  y: %f  z:%f]",w(0),w(1),w(2));
}

int32_t pwmMapping(double pwm){
	return (int32_t)(65535.*pwm/(1./pwm_freq*1000000.));
}

void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4, double pwm5, double pwm6, double pwm7, double pwm8){
	PWMs_cmd.data.resize(8);
	PWMs_cmd.data[0] = pwm1;
	PWMs_cmd.data[1] = pwm2;
	PWMs_cmd.data[2] = pwm3;
	PWMs_cmd.data[3] = pwm4;
	PWMs_cmd.data[4] = pwm5;
	PWMs_cmd.data[5] = pwm6;
	PWMs_cmd.data[6] = pwm7;
	PWMs_cmd.data[7] = pwm8;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(pwm1);
	PWMs_val.data[1] = pwmMapping(pwm2);
	PWMs_val.data[2] = pwmMapping(pwm3);
	PWMs_val.data[3] = pwmMapping(pwm4);
	PWMs_val.data[4] = pwmMapping(pwm5);
	PWMs_val.data[5] = pwmMapping(pwm6);
	PWMs_val.data[6] = pwmMapping(pwm7);
	PWMs_val.data[7] = pwmMapping(pwm8);
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}

void pwm_Max(){
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(2000.);
	PWMs_val.data[1] = pwmMapping(2000.);
	PWMs_val.data[2] = pwmMapping(2000.);
	PWMs_val.data[3] = pwmMapping(2000.);
	PWMs_val.data[4] = pwmMapping(2000.);
	PWMs_val.data[5] = pwmMapping(2000.);
	PWMs_val.data[6] = pwmMapping(2000.);
	PWMs_val.data[7] = pwmMapping(2000.);
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;
}

void pwm_Kill(){
	PWMs_cmd.data.resize(8);
	PWMs_cmd.data[0] = 1000;
	PWMs_cmd.data[1] = 1000;
	PWMs_cmd.data[2] = 1000;
	PWMs_cmd.data[3] = 1000;
	PWMs_cmd.data[4] = 1000;
	PWMs_cmd.data[5] = 1000;
	PWMs_cmd.data[6] = 1000;
	PWMs_cmd.data[7] = 1000;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1000.);
	PWMs_val.data[1] = pwmMapping(1000.);
	PWMs_val.data[2] = pwmMapping(1000.);
	PWMs_val.data[3] = pwmMapping(1000.);
	PWMs_val.data[4] = pwmMapping(1000.);
	PWMs_val.data[5] = pwmMapping(1000.);
	PWMs_val.data[6] = pwmMapping(1000.);
	PWMs_val.data[7] = pwmMapping(1000.);
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}

void pwm_Arm(){
	PWMs_cmd.data.resize(8);
	PWMs_cmd.data[0] = 1200;
	PWMs_cmd.data[1] = 1200;
	PWMs_cmd.data[2] = 1200;
	PWMs_cmd.data[3] = 1200;
	PWMs_cmd.data[4] = 1200;
	PWMs_cmd.data[5] = 1200;
	PWMs_cmd.data[6] = 1200;
	PWMs_cmd.data[7] = 1200;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1200.);
	PWMs_val.data[1] = pwmMapping(1200.);
	PWMs_val.data[2] = pwmMapping(1200.);
	PWMs_val.data[3] = pwmMapping(1200.);
	PWMs_val.data[4] = pwmMapping(1200.);
	PWMs_val.data[5] = pwmMapping(1200.);
	PWMs_val.data[6] = pwmMapping(1200.);
	PWMs_val.data[7] = pwmMapping(1200.);
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}
void pwm_Calibration(){
	if(Sbus[4]>1500) pwm_Arm();
	else pwm_Kill();
}

void pid_Gain_Setting(){
	if(Sbus[8]<=1500){
		Pa = conv_Pa;
		Ia = conv_Ia;
		Da = conv_Da;

		Py = conv_Py;
		Dy = conv_Dy;

		Pz = conv_Pz;
		Iz = conv_Iz;
		Dz = conv_Dz;
		
		Pv = conv_Pv;
		Iv = conv_Iv;
		Dv = conv_Dv;

		Pp = conv_Pp;
		Ip = conv_Ip;
		Dp = conv_Dp;
	}
	else{
		Pa = tilt_Pa;
		Ia = tilt_Ia;
		Da = tilt_Da;

		Py = tilt_Py;
		Dy = tilt_Dy;

		Pz = tilt_Pz;
		Iz = tilt_Iz;
		Dz = tilt_Dz;
		
		Pv = tilt_Pv;
		Iv = tilt_Iv;
		Dv = tilt_Dv;

		Pp = tilt_Pp;
		Ip = tilt_Ip;
		Dp = tilt_Dp;
	}
	//ROS_INFO("%.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf ",Pa, Ia, Da, Py, Dy, Pz, Iz, Dz, Pp, Ip, Dp);
}

void disturbance_Observer(){
	//DOB------------------------------------------------------------------------------
	//Nominal transfer function : q/tau = 1/Js^2    Q - 3rd order butterworth filter
	//Roll
	//Q*(Js^2) transfer function to state space 
	x_dot_r1 = -2*fq_cutoff*x_r1-2*pow(fq_cutoff,2)*x_r2-pow(fq_cutoff,3)*x_r3+imu_rpy.x;
	x_dot_r2 = x_r1;
	x_dot_r3 = x_r2;
    x_r1 += x_dot_r1*delta_t.count(); 
	x_r2 += x_dot_r2*delta_t.count(); 
	x_r3 += x_dot_r3*delta_t.count(); 
	double tauhat_r = J_x*pow(fq_cutoff,3)*x_r1;

	//Q transfer function to state space
	y_dot_r1 = -2*fq_cutoff*y_r1-2*pow(fq_cutoff,2)*y_r2-pow(fq_cutoff,3)*y_r3+tautilde_r_d;
	y_dot_r2 = y_r1;
	y_dot_r3 = y_r2;
	y_r1 += y_dot_r1*delta_t.count();
	y_r2 += y_dot_r2*delta_t.count();
	y_r3 += y_dot_r3*delta_t.count();
	double Qtautilde_r = pow(fq_cutoff,3)*y_r3;

	dhat_r = tauhat_r - Qtautilde_r;


	//Pitch
	//Q*(Js^2) transfer function to state space 
	x_dot_p1 = -2*fq_cutoff*x_p1-2*pow(fq_cutoff,2)*x_p2-pow(fq_cutoff,3)*x_p3+imu_rpy.y;
	x_dot_p2 = x_p1;
	x_dot_p3 = x_p2;
	x_p1 += x_dot_p1*delta_t.count(); 
	x_p2 += x_dot_p2*delta_t.count(); 
	x_p3 += x_dot_p3*delta_t.count(); 
	double tauhat_p = J_y*pow(fq_cutoff,3)*x_p1;

	//Q transfer function to state space
	y_dot_p1 = -2*fq_cutoff*y_p1-2*pow(fq_cutoff,2)*y_p2-pow(fq_cutoff,3)*y_p3+tautilde_p_d;
	y_dot_p2 = y_p1;
	y_dot_p3 = y_p2;
	y_p1 += y_dot_p1*delta_t.count();
	y_p2 += y_dot_p2*delta_t.count();
	y_p3 += y_dot_p3*delta_t.count();
	double Qtautilde_p = pow(fq_cutoff,3)*y_p3;

	dhat_p = tauhat_p - Qtautilde_p;


	//Yaw
	//Q*(Js^2) transfer function to state space 
	x_dot_y1 = -2*fq_cutoff*x_y1-2*pow(fq_cutoff,2)*x_y2-pow(fq_cutoff,3)*x_y3+imu_rpy.z;
	x_dot_y2 = x_y1;
	x_dot_y3 = x_y2;
    x_y1 += x_dot_y1*delta_t.count(); 
	x_y2 += x_dot_y2*delta_t.count(); 
	x_y3 += x_dot_y3*delta_t.count(); 
	double tauhat_y = J_z*pow(fq_cutoff,3)*x_y1;

	//Q transfer function to state space
	y_dot_y1 = -2*fq_cutoff*y_y1-2*pow(fq_cutoff,2)*y_y2-pow(fq_cutoff,3)*y_y3+tautilde_y_d;
	y_dot_y2 = y_y1;
	y_dot_y3 = y_y2;
	y_y1 += y_dot_y1*delta_t.count();
	y_y2 += y_dot_y2*delta_t.count();
	y_y3 += y_dot_y3*delta_t.count();
	double Qtautilde_y = pow(fq_cutoff,3)*y_y3;

	double dhat_y = tauhat_y - Qtautilde_y;
	dhat.x = dhat_r;
	dhat.y = dhat_p;
	dhat.z = dhat_y;

	//tautilde_y_d = tau_y_d - dhat_y;
    tautilde_y_d = tau_y_d;
	//--------------------------------------------------------------------------------------
}

void sine_wave_vibration(){
	vibration1 = Amp_Z*sin(pass_freq1*time_count);
	vibration2 = Amp_XY*sin(pass_freq2*time_count);
	sine_wave.x = vibration1;
	sine_wave.y = vibration2;
	time_count += delta_t.count();
}

//-------------------------------------------------------

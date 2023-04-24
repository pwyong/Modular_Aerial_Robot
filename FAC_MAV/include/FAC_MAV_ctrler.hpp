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
geometry_msgs::Vector3 bias_gradient_data;
geometry_msgs::Vector3 filtered_bias_gradient_data;

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
bool DOB_mode = false;

//State
bool hovering = false;
bool loading = false;
double hovering_time_count = 0;
double hovering_force = 0;
double hovering_count = 0;
double loading_time_count = 0;
double loading_force = 0;
double unloading_time_count=0;
double loading_count = 0;
bool x_c_convergence = false;
bool y_c_convergence = false;
bool z_c_convergence = false;
double x_c_convergence_time_count = 0;
double y_c_convergence_time_count = 0;
double z_c_convergence_time_count = 0;
bool estimating = false;
double estimation_timer = 0;
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
static double mass = 2.405;//(Kg)


//Propeller constants(DJI E800(3510 motors + 620S ESCs))
static double b_over_k_ratio = 0.01;//F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
//--------------------------------------------------------

//General parameters======================================

static double pi = 3.141592;//(rad)
static double g = 9.80665;//(m/s^2)

static double rp_limit = 0.3;//(rad)
static double y_vel_limit = 0.01;//(rad/s)
static double y_d_tangent_deadzone = (double)0.05 * y_vel_limit;//(rad/s)
static double T_limit = 21;//(N)
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
double fq_cutoff=5.0;//Q filter Cut-off frequency

// Nominal MoI
double J_x = 0.001;
double J_y = 0.001;
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
ros::Publisher bias_gradient;
ros::Publisher filtered_bias_gradient;
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

double MoI_x_hat = 0.005;
double MoI_y_hat = 0.005;
double G_XY = 0.3;
double G_Z = 1.0;

double x_c_init = 0.0;
double y_c_init = 0.0;
double z_c_init = 0.0;
double bias_x_c = 0;
double bias_y_c = 0;
double bias_z_c = 0;
double x_c_limit = 0.04;
double y_c_limit = 0.04;
double z_c_limit = 0.1;

//Bandpass filter parameter
double Q_factor=20.0;
double pass_freq1=5.0;
double pass_freq2=3.0;

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
double Amp_XY=1.0;
double Amp_Z=1.0;

//gradient LPF
double xy_cutoff_freq = 0.2;
double x_grad_x_dot=0;
double x_grad_x=0;
double filtered_grad_x=0;

double x_grad_y_dot=0;
double x_grad_y=0;
double filtered_grad_y=0;

double z_cutoff_freq = 0.2;
double x_grad_z_dot=0;
double x_grad_z=0;
double filtered_grad_z=0;
//-----------------------------------------------------
//Accelerometer LPF------------------------------------
double x_ax_dot = 0;
double x_ay_dot = 0;
double x_ax = 0;
double x_ay = 0;
double accel_cutoff_freq = 5.0;
//-----------------------------------------------------

//Trajectory------------------------------------------
double Amp = 1.5;
double Tp = 7.0;
double traj_timer = 0;
bool trajectory_flag = false;
//----------------------------------------------------
template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

void state_init(){
	hovering = false;
	loading = false;
	tilt_mode = false;
	x_c_convergence = false;
	y_c_convergence = false;
	z_c_convergence = false;
	estimating = false;
	ESC_control = false;
	x_c_hat=x_c_init;
	y_c_hat=y_c_init;
	z_c_hat=z_c_init;
	bias_x_c=0;
	bias_y_c=0;
	bias_z_c=0;
	hovering_time_count=0;
	loading_time_count=0;
	x_c_convergence_time_count=0;
	y_c_convergence_time_count=0;
	z_c_convergence_time_count=0;
	unloading_time_count=0;
	hovering_force = 0;
	loading_force = 0;
	hovering_count = 0;
	loading_count = 0;
	X_d_base = pos.x;
	Y_d_base = pos.y;
	traj_timer = 0;
	estimation_timer = 0;	
	DOB_mode=false;

	x_grad_x_dot=0;
	x_grad_x=0;
	filtered_grad_x=0;

	x_grad_y_dot=0;
	x_grad_y=0;
	filtered_grad_y=0;

	x_grad_z_dot=0;
	x_grad_z=0;
	filtered_grad_z=0;
	time_count=0;

	x_dot_11=0;
	x_dot_12=0;
	x_11=0;
	x_12=0;
	y_11=0;
	x_dot_21=0;
	x_dot_22=0;
	x_21=0;
	x_22=0;
	y_21=0;
	x_dot_31=0;
	x_dot_32=0;
	x_31=0;
	x_32=0;
	y_31=0;
}

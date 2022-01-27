#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>
#include <vector>
#include <cmath>
#include <cstdio>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Imu.h>
#include "t3_mav_controller/t3_mav_controller.h"

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

#include "nav_msgs/Odometry.h"

double freq=200;//controller loop frequency
double pwm_freq=417.3;//pwm signal frequency
int16_t Sbus[8];
int16_t PWM_d;
int16_t loop_time;
std_msgs::Int16MultiArray PWMs_cmd;
std_msgs::Int32MultiArray PWMs_val;
geometry_msgs::Quaternion Force; //Only for plotting in Python (Hard to use Float32MultiArray)

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
geometry_msgs::Vector3 ang_vel;
geometry_msgs::Quaternion t265_quat;
geometry_msgs::Quaternion rot;
geometry_msgs::Quaternion desired_value;
geometry_msgs::Vector3 integrator;
geometry_msgs::Vector3 desired_pos;

geometry_msgs::Vector3 t265_att;
geometry_msgs::Vector3 filtered_angular_rate;
std_msgs::Float32 altitude_d;
bool servo_sw=false;
double theta1_command, theta2_command;
bool start_flag=false;
bool tilting_flag=false;
//Thruster_cmd

double F1 = 0;//desired propeller 1 force
double F2 = 0;//desired propeller 2 force
double F3 = 0;//desired propeller 3 force
double F4 = 0;//desired propeller 4 force

double e_r_i = 0;//roll error integration
double e_p_i = 0;//pitch error integration
double e_x_i = 0;//x position error integration
double e_y_i = 0;//y position error integration
double e_z_i = 0;//altitude error integration

double tau_r_d = 0;//roll  desired torque (N.m)
double tau_p_d = 0;//pitch desired torque(N.m)

double Thrust_d = 0;//altitiude desired thrust(N)
double prev_z=0;
double z_vel=0;
//ud_cmd

double r_d = 0;//desired roll angle
double p_d = 0;//desired pitch angle
double y_d = 0;//desired yaw angle
double y_d_tangent = 0;//yaw increment tangent
double T_d = 0;//desired thrust
double z_d = 0;//desired altitude

double X_d = 0;
double Y_d = 0;
double X_d_base = 0;
double Y_d_base = 0;
double X_ddot_d = 0;
double Y_ddot_d = 0;

double alpha = 0;
double beta = 0;
//--------------------------------------------------------

//General dimensions

static double l_arm = 0.109;// m // diagonal length between thruster : 218mm;
static double l_servo = -0.045;
static double mass = 1.992;//(Kg)
double initial_z = 0;


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
static double servo_limit=0.3;
static double XY_ddot_limit=2;
static double alpha_beta_limit=1;

double x_c=0.0;
double y_c=0.0;
double z_c=0.0;
//--------------------------------------------------------

//Control gains===========================================

//integratior(PID) limitation
double integ_limit=10;
double z_integ_limit=100;
double pos_integ_limit=10;

//Roll, Pitch PID gains
double Pa=3.5;
double Ia=0.4;
double Da=0.5;


//Yaw PID gains
double Py=2.0;
double Dy=0.1;

//Altitude PID gains
double Pz=16.0;
double Iz=5.0;
double Dz=15.0;

//
double Pp=3.0;
double Ip=0.1;
double Dp=5.0;
//--------------------------------------------------------

//Servo angle=============================================
double theta1=0,theta2=0;
//--------------------------------------------------------

//Function------------------------------------------------
template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void rpyT_ctrl();
void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des);
double Force_to_PWM(double F);
void jointstateCallback(const sensor_msgs::JointState& msg);
void imu_Callback(const sensor_msgs::Imu& msg);
sensor_msgs::JointState servo_msg_create(double rr, double rp);
void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array);
void loopCallback(const std_msgs::Int16& time);
void posCallback(const geometry_msgs::Vector3& msg);
void rotCallback(const geometry_msgs::Quaternion& msg);
void filterCallback(const sensor_msgs::Imu& msg);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void setCM();
void publisherSet();
int32_t pwmMapping(double pwm);
void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4);
void pwm_Kill();
//-------------------------------------------------------

//Publisher Group--------------------------------------
ros::Publisher PWMs;
ros::Publisher goal_dynamixel_position_;
ros::Publisher euler;
ros::Publisher desired_angle;
ros::Publisher desired_altitude;
ros::Publisher Forces;
ros::Publisher desired_torque;
ros::Publisher i_result;
ros::Publisher linear_velocity;
ros::Publisher angular_velocity;
ros::Publisher PWM_generator;
ros::Publisher desired_position;
ros::Publisher position;
//----------------------------------------------------

//Control Matrix---------------------------------------
Eigen::Matrix4d CM;
Eigen::Matrix4d invCM;
Eigen::Vector4d u;
Eigen::Vector4d F;
//-----------------------------------------------------

//Linear_velocity--------------------------------------
Eigen::Vector3d cam_v;
Eigen::Matrix3d R_v;
Eigen::Vector3d v;
//-----------------------------------------------------
//Angular_velocity-------------------------------------
Eigen::Vector3d cam_w;
Eigen::Matrix3d R_w;
Eigen::Vector3d w;
//-----------------------------------------------------
//Attitude--------------------------------------
Eigen::Vector3d cam_att;
Eigen::Matrix3d R_a;
Eigen::Vector3d t4_att;
//-----------------------------------------------------
double iterator = 0;

int main(int argc, char **argv){
	
    	ros::init(argc, argv,"t3_mav_controller");

    	std::string deviceName;
    	ros::NodeHandle params("~");
    	params.param<std::string>("device", deviceName, "/gx5");

    	ros::NodeHandle nh;

	//Loading gains from the "t3_mav_controller.launch" file
		//integratior(PID) limitation
		integ_limit=nh.param<double>("attitude_integ_limit",10);
		z_integ_limit=nh.param<double>("altitude_integ_limit",100);
		pos_integ_limit=nh.param<double>("position_integ_limit",10);

		//Roll, Pitch PID gains
		Pa=nh.param<double>("attitude_rp_P_gain",3.5);
		Ia=nh.param<double>("attitude_rp_I_gain",0.4);
		Da=nh.param<double>("attitude_rp_D_gain",0.5);

		//Yaw PID gains
		Py=nh.param<double>("attitude_y_P_gain",2.0);
		Dy=nh.param<double>("attitude_y_D_gain",0.1);

		//Altitude PID gains
		Pz=nh.param<double>("altitude_P_gain",16.0);
		Iz=nh.param<double>("altitude_I_gain",5.0);
		Dz=nh.param<double>("altitude_D_gain",15.0);

		//Position PID gains
		Pp=nh.param<double>("position_P_gain",3.0);
		Ip=nh.param<double>("position_I_gain",0.1);
		Dp=nh.param<double>("position_D_gain",5.0);

		//Center of Mass
		x_c=nh.param<double>("x_center_of_mass",0.0);
		y_c=nh.param<double>("y_center_of_mass",0.0);
		z_c=nh.param<double>("z_center_of_mass",0.0);
	//----------------------------------------------------------
	
    	PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1);
	PWM_generator = nh.advertise<std_msgs::Int32MultiArray>("/command",1);
    	goal_dynamixel_position_  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",100);
	euler = nh.advertise<geometry_msgs::Vector3>("angle",1);
	desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",100);
	desired_altitude = nh.advertise<std_msgs::Float32>("desired_altitude",100);
	Forces = nh.advertise<geometry_msgs::Quaternion>("Forces",100);
	desired_torque = nh.advertise<geometry_msgs::Quaternion>("torque_d",100);
	i_result = nh.advertise<geometry_msgs::Vector3>("i_result",100);
	linear_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel",100);
	angular_velocity = nh.advertise<geometry_msgs::Vector3>("gyro",100);
	desired_position = nh.advertise<geometry_msgs::Vector3>("pos_d",100);
	position = nh.advertise<geometry_msgs::Vector3>("pos",100);

    	ros::Subscriber dynamixel_state = nh.subscribe("joint_states",100,jointstateCallback,ros::TransportHints().tcpNoDelay());
    	ros::Subscriber att = nh.subscribe("/gx5/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay());
    	ros::Subscriber rc_in = nh.subscribe("/sbus",100,sbusCallback,ros::TransportHints().tcpNoDelay());
	//ros::Subscriber loop_timer = nh.subscribe("/loop",100,loopCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_pos=nh.subscribe("/t265_pos",100,posCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_rot=nh.subscribe("/t265_rot",100,rotCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_odom=nh.subscribe("/rs_t265/odom/sample",100,t265OdomCallback,ros::TransportHints().tcpNoDelay());
	
	ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    	ros::spin();
    	return 0;
}

void publisherSet(){

	setCM(); // Setting Control Matrix
    	//Publish data	
	if(Sbus[8]>1500){
		
	}
	else{
		theta1_command=0.0;
		theta2_command=0.0;
	}
	if(Sbus[6]<=1500){
		X_d_base=pos.x;
		Y_d_base=pos.y;
		X_d = X_d_base;
		Y_d = Y_d_base;
		e_x_i=0;
		e_y_i=0;
		
	}

	if(Sbus[4]<1500){	
		y_d=cam_att(2);	//[J]This line ensures that yaw desired right after disabling the kill switch becomes current yaw attitude
		initial_z=pos.z;
		e_r_i = 0;
		e_p_i = 0;
		e_z_i = 0;
		e_x_i=0;
		e_y_i=0;
		start_flag=false;
		pwm_Kill();	
	}
	else{
		// ROS_INFO("r:%lf, p:%lf, y:%lf T:%lf", r_d, p_d, y_d, T_d);
		rpyT_ctrl();	
	}
	
	angle_d.x=r_d;
	angle_d.y=p_d;
	angle_d.z=y_d;
	desired_pos.x = X_d;
	desired_pos.y = Y_d;
	desired_pos.z = z_d;
	altitude_d.data=z_d;
	PWMs.publish(PWMs_cmd);
	euler.publish(imu_rpy);
	desired_angle.publish(angle_d);
	Forces.publish(Force);
	desired_altitude.publish(altitude_d);
	goal_dynamixel_position_.publish(servo_msg_create(-theta2_command,-theta1_command));
	desired_torque.publish(desired_value);
	i_result.publish(integrator);
	linear_velocity.publish(lin_vel);
	PWM_generator.publish(PWMs_val);
	desired_position.publish(desired_pos);
	position.publish(pos);
	// ROS_INFO("%d %d %d %d",PWMs_cmd.data[0],PWMs_cmd.data[1],PWMs_cmd.data[2],PWMs_cmd.data[3]);
	// ROS_INFO("loop time : %f",((double)end.sec-(double)begin.sec)+((double)end.nsec-(double)begin.nsec)/1000000000.);

}

void setCM(){
	CM << y_c+(l_servo+z_c)*theta1, l_arm+y_c, y_c+(l_servo+z_c)*theta1, y_c-l_arm,
	      l_arm-x_c, -x_c-(l_servo-z_c)*theta2, -l_arm-x_c, -x_c-(l_servo-z_c)*theta2,
	      -b_over_k_ratio+(l_arm-x_c)*theta1, b_over_k_ratio-(l_arm+y_c)*theta2, -b_over_k_ratio-(l_arm+x_c)*theta1, b_over_k_ratio+(l_arm-y_c)*theta2,
	      -1.0, -1.0, -1.0, -1.0;
	invCM=CM.inverse();
}

void rpyT_ctrl() {
	

	y_d_tangent=y_vel_limit*(((double)Sbus[0]-(double)1500)/(double)500);
	if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
	y_d+=y_d_tangent;
	
	z_d=altitude_limit*(((double)Sbus[2]-(double)1500)/(double)500)+altitude_limit;
	T_d =-T_limit*(((double)Sbus[2]-(double)1500)/(double)500)-T_limit;

	double e_r = 0;
	double e_p = 0;
	double e_y = 0;
	double e_X = 0;
	double e_Y = 0;
	double e_z = z_d - pos.z;
	//double delta_z=pos.z-prev_z;
	
	theta1_command = 0.0;
	theta2_command = 0.0;	

	if(Sbus[6]>1500){
		X_d = X_d_base - XY_limit*(((double)Sbus[1]-(double)1500)/(double)500);
		Y_d = Y_d_base + XY_limit*(((double)Sbus[3]-(double)1500)/(double)500);
		
		e_X = X_d - pos.x;
		e_Y = Y_d - pos.y;
		e_x_i += e_X * ((double)1 / freq);
		if (fabs(e_x_i) > pos_integ_limit) e_x_i = (e_x_i / fabs(e_x_i)) * pos_integ_limit;
		e_y_i += e_Y * ((double)1 / freq);
		if (fabs(e_y_i) > pos_integ_limit) e_y_i = (e_y_i / fabs(e_y_i)) * pos_integ_limit;
	
		X_ddot_d = Pp * e_X + Ip * e_x_i - Dp * v(0);
		Y_ddot_d = Pp * e_Y + Ip * e_y_i - Dp * v(1);	
		if(fabs(X_ddot_d) > XY_ddot_limit) X_ddot_d = (X_ddot_d/fabs(X_ddot_d))*XY_ddot_limit;
		if(fabs(Y_ddot_d) > XY_ddot_limit) Y_ddot_d = (Y_ddot_d/fabs(Y_ddot_d))*XY_ddot_limit;

		alpha=(-sin(cam_att(2))*X_ddot_d+cos(cam_att(2))*Y_ddot_d)/g;
		beta=(-cos(cam_att(2))*X_ddot_d-sin(cam_att(2))*Y_ddot_d)/g;
		if(fabs(alpha) > alpha_beta_limit) alpha = (alpha/fabs(alpha))*alpha_beta_limit;
		if(fabs(beta) > alpha_beta_limit) beta = (beta/fabs(beta))*alpha_beta_limit;

		if(Sbus[8]>1500){
			r_d = 0.0;
			p_d = 0.0;
			theta1_command = mass*Y_ddot_d/(F(0)+F(2)); //theta1 = F_y/(F1+F3)
			theta2_command = -mass*X_ddot_d/(F(1)+F(3)); //theta2 = -F_x/(F2+F4)
			if(fabs(theta1_command)>servo_limit) theta1_command = (theta1_command/fabs(theta1_command))*servo_limit;
			if(fabs(theta2_command)>servo_limit) theta2_command = (theta2_command/fabs(theta2_command))*servo_limit;
		}
		else{
			r_d = asin(alpha);
			p_d = asin(beta/cos(imu_rpy.x));
			if(fabs(r_d)>rp_limit) r_d = (r_d/fabs(r_d))*rp_limit;
			if(fabs(p_d)>rp_limit) p_d = (p_d/fabs(p_d))*rp_limit;
			//ROS_INFO("Position Control!!");
		}
	}
	else{
		r_d=rp_limit*(((double)Sbus[3]-(double)1500)/(double)500);
		p_d=rp_limit*(((double)Sbus[1]-(double)1500)/(double)500);
		//ROS_INFO("Attidue Control!!");
	}
	
	e_r = r_d - imu_rpy.x;
	e_p = p_d - imu_rpy.y;
	e_y = y_d - cam_att(2);

	
	e_r_i += e_r * ((double)1 / freq);
	if (fabs(e_r_i) > integ_limit)	e_r_i = (e_r_i / fabs(e_r_i)) * integ_limit;
	e_p_i += e_p * ((double)1 / freq);
	if (fabs(e_p_i) > integ_limit)	e_p_i = (e_p_i / fabs(e_p_i)) * integ_limit;
	e_z_i += e_z * ((double)1 / freq);	
	if (fabs(e_z_i) > z_integ_limit) e_z_i = (e_z_i / fabs(e_z_i)) * z_integ_limit;
	integrator.x=e_x_i;
	integrator.y=e_y_i;
	integrator.z=e_z_i;

	tau_r_d = Pa * e_r + Ia * e_r_i + Da * (-imu_ang_vel.x);// - (double)0.5;
	tau_p_d = Pa * e_p + Ia * e_p_i + Da * (-imu_ang_vel.y);// + (double)0.2; 

	if(Sbus[5]>1500){
		Thrust_d =-(Pz * e_z + Iz * e_z_i - Dz * (-v(2)) + mass*g);
		// ROS_INFO("Altitude Control!!");
	}
	else{
		Thrust_d=T_d;
		// ROS_INFO("Manual Thrust!!");
	}

	double tau_y_d = Py * e_y + Dy * (-imu_ang_vel.z);

	u << tau_r_d, tau_p_d, tau_y_d, Thrust_d;

	//ROS_INFO("xvel:%lf, yvel:%lf, zvel:%lf", imu_ang_vel.x, imu_ang_vel.y, imu_ang_vel.z);
	// ROS_INFO("tr:%lf, tp:%lf, ty:%lf, Thrust_d:%lf", tau_r_d, tau_p_d, tau_y_d, Thrust_d);
	// ROS_INFO("%f",Dz*freq*delta_z);
	ud_to_PWMs(tau_r_d, tau_p_d, tau_y_d, Thrust_d);
	desired_value.x=tau_r_d;
	desired_value.y=tau_p_d;
	desired_value.z=tau_y_d;
	desired_value.w=Thrust_d;
}

 

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des) {

	F1 = +((double)0.5 / l_arm) * tau_p_des - ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
	F2 = +((double)0.5 / l_arm) * tau_r_des + ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
	F3 = -((double)0.5 / l_arm) * tau_p_des - ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
	F4 = -((double)0.5 / l_arm) * tau_r_des + ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;

    	F=invCM*u;
	//ROS_INFO("F1:%lf, F2:%lf, F3:%lf, F4:%lf", F(0), F(1), F(2), F(3));
	PWMs_cmd.data.resize(4);
	// PWMs_cmd.data[0] = 1000;
	// PWMs_cmd.data[1] = 1000;
	// PWMs_cmd.data[2] = 1000;
	// PWMs_cmd.data[3] = 1000;
	
	//PWMs_cmd.data[0] = Force_to_PWM(F1);
	//PWMs_cmd.data[1] = Force_to_PWM(F2);
	//PWMs_cmd.data[2] = Force_to_PWM(F3);
	//PWMs_cmd.data[3] = Force_to_PWM(F4);
	
	pwm_Command(Force_to_PWM(F(0)), Force_to_PWM(F(1)), Force_to_PWM(F(2)), Force_to_PWM(F(3)));

	Force.x=PWMs_val.data[0];
	Force.y=PWMs_val.data[1];
	Force.z=PWMs_val.data[2];
	Force.w=PWMs_val.data[3];

	// ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
	// ROS_INFO("%f 1:%d, 2:%d, 3:%d, 4:%d",z_d,PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
}

 

double Force_to_PWM(double F) {
	double pwm;
	double param1 = 1111.07275742670;
	double param2 = 44543.2632092715;
	double param3 = 6112.46876873481;
	//Force=0.0000224500839846839*PWM^2-0.049887353434648*PWM+27.577014233466
	//PWM=1111.07275742670+(44543.2632092715*Force+6112.46876873481)^0.5
	if(param2*F+param3>0){
		pwm = param1 + sqrt(param2 * F + param3);
	}
	else pwm = param1;
	if (pwm > 1900)	pwm = 1900;
	if(Sbus[5]>1500){
		if(initial_z>=0){
			if(z_d<initial_z && !start_flag) {
				pwm=param1;
			}
			else if(z_d>initial_z) start_flag=true;
		}
		else pwm=param1;
	}
	return pwm;
}

void jointstateCallback(const sensor_msgs::JointState& msg){
    rac_servo_value=msg;
	theta1=-msg.position[1];
	theta2=-msg.position[0];
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
    imu_rpy.z=cam_att(2);
    
	// ROS_INFO("imuCallback time : %f",(((double)ros::Time::now().sec-(double)imuTimer.sec)+((double)ros::Time::now().nsec-(double)imuTimer.nsec)/1000000000.));
	imuTimer = ros::Time::now();
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

	return servo_msg;
}

void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array){
	for(int i=0;i<9;i++){
		Sbus[i]=map<int16_t>(array->data[i], 352, 1696, 1000,2000);
	}
	// PWM_d=Sbus[2];
    // ROS_INFO("SBUS - [%d, %d, %d, %d, %d, %d, %d, %d]",Sbus[0], Sbus[1], Sbus[2], Sbus[3], Sbus[4], Sbus[5], Sbus[6], Sbus[7]);
    
	
	return;
}


void loopCallback(const std_msgs::Int16& time){
	loop_time=time.data;
}

ros::Time posTimer;
void posCallback(const geometry_msgs::Vector3& msg){
	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=-msg.z;

	// ROS_INFO("Translation - [x: %f  y:%f  z:%f]",pos.x,pos.y,pos.z);
	/*double dt = ((double)ros::Time::now().sec-(double)posTimer.sec)+((double)ros::Time::now().nsec-(double)posTimer.nsec)/1000000000.;
	z_vel = (pos.z-prev_z)/dt;
	posTimer = ros::Time::now();
	prev_z = pos.z;*/
	//ROS_INFO("z_vel : %f",z_vel);

}

void rotCallback(const geometry_msgs::Quaternion& msg){
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;

	/*t265_att.x=atan2((rot.y*rot.z+rot.w*rot.x),1.-2.*(rot.z*rot.z+rot.w*rot.w));
	t265_att.y=asin(2.*(rot.y*rot.w-rot.x*rot.z));
	t265_att.z=atan2((rot.y*rot.x+rot.z*rot.w),1.-2.*(rot.w*rot.w+rot.x*rot.x));*/
	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
	//ROS_INFO("Attitude - [r: %f  p: %f  y:%f]",t4_att(0),t4_att(1),t4_att(2));	
}



void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	t265_lin_vel=msg->twist.twist.linear;
	t265_ang_vel=msg->twist.twist.angular;
	t265_quat=msg->pose.pose.orientation;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(cam_att(0),cam_att(1),cam_att(2));
	cam_v << t265_lin_vel.x, t265_lin_vel.y, t265_lin_vel.z;
	R_v << cos(pi/4.), -sin(pi/4.),  0.,
 	       sin(pi/4.),  cos(pi/4.),  0.,
	                0.,           0.,  1.;

	v = R_v*cam_v;
        //t4_att = R_a*cam_att;
	lin_vel.x=v(0);
	lin_vel.y=v(1);
	lin_vel.z=-v(2);
	//ROS_INFO("Attitude - [r: %f  p: %f  y:%f]",cam_att(0),cam_att(1),cam_att(2));
	//ROS_INFO("Linear_velocity - [x: %f  y: %f  z:%f]",v(0),v(1),v(2));
	//ROS_INFO("Angular_velocity - [x: %f  y: %f  z:%f]",w(0),w(1),w(2));
}

int32_t pwmMapping(double pwm){
	return (int32_t)(65535.*pwm/(1./pwm_freq*1000000.));
}

void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4){
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(pwm1);
	PWMs_val.data[1] = pwmMapping(pwm2);
	PWMs_val.data[2] = pwmMapping(pwm3);
	PWMs_val.data[3] = pwmMapping(pwm4);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
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
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1000.);
	PWMs_val.data[1] = pwmMapping(1000.);
	PWMs_val.data[2] = pwmMapping(1000.);
	PWMs_val.data[3] = pwmMapping(1000.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}


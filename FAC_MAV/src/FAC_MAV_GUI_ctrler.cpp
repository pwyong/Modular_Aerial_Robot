//2022.05.16 Coaxial-Octorotor version
//2022.06.23 Ground Station Application

// 8: 중앙서보, 6 : 포지션제어, 4: 킬, 5 : 고도제어
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
#include "FAC_MAV/ArmService.h" 
#include "FAC_MAV/KillService.h" 
#include "FAC_MAV/PosCtrlService.h" 
#include "FAC_MAV/HoverService.h"
#include "FAC_MAV/FAC_HoverService.h"
#include "FAC_MAV/TiltService.h"

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
geometry_msgs::Vector3 F_total;
geometry_msgs::Vector3 torque_d;
geometry_msgs::Vector3 force_d;
std_msgs::Float32MultiArray force_cmd;

geometry_msgs::Vector3 t265_att;
geometry_msgs::Vector3 filtered_angular_rate;
std_msgs::Float32 altitude_d;
std_msgs::Float32 battery_voltage_msg;
std_msgs::Float32 battery_real_voltage;
bool servo_sw=false;
double theta1_command, theta2_command;
bool start_flag=false;
bool tilting_flag=false;
//Thruster_cmd

double F1 = 0;//desired propeller 1 force
double F2 = 0;//desired propeller 2 force
double F3 = 0;//desired propeller 3 force
double F4 = 0;//desired propeller 4 force
double F5 = 0;//desired propeller 5 force
double F6 = 0;//desired propeller 6 force
double F7 = 0;//desired propeller 7 force
double F8 = 0;//desired propeller 8 force

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

double yaw_prev = 0;
double yaw_now = 0;
double base_yaw = 0;
int yaw_rotate_count = 0;

//-----------------------GUI control parameter---------------------------//
double Landing_time = 2.0; //put landing time (sec)
double Hovering_time = 1.0; //put Hovering time (sec)
double X_Speed = 0.2; //desired value change speed of x_axis
double Y_Speed = 0.2; //desired value change speed of y_axis
double yaw_Speed = 0.2; //desired value change speed of yaw
double Z_Speed = 0.2; //desired value change speed of z_axis
//---------------------------------------------------------//
double Landing_Inc = 1 / (freq * Landing_time);
double Hovering_Inc = 1 / (freq * Hovering_time);
double X_Inc = X_Speed / freq;
double Y_Inc = Y_Speed / freq;
double yaw_Inc = yaw_Speed / freq;
double z_Inc = Z_Speed / freq;
double X_Goal = 0;
double Y_Goal = 0;
double y_Goal = 0;
double z_Goal = 0;
//--------------------------------------------------------

//General dimensions

static double l_arm = 0.109;// m // diagonal length between thruster : 218mm;
static double l_servo = -0.015;
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
static double hardware_servo_limit=0.3;
static double XY_ddot_limit=1;
static double alpha_beta_limit=1;
static double servo_command_limit = 0.1;
static double tau_y_limit = 0.3;

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

//Conventional Flight Mode Control Gains
double conv_Pa, conv_Ia, conv_Da;
double conv_Py, conv_Dy;
double conv_Pz, conv_Iz, conv_Dz;
double conv_Pp, conv_Ip, conv_Dp;

//Tilt Flight Mode Control Gains
double tilt_Pa, tilt_Ia, tilt_Da;
double tilt_Py, tilt_Dy;
double tilt_Pz, tilt_Iz, tilt_Dz;
double tilt_Pp, tilt_Ip, tilt_Dp;
//--------------------------------------------------------

//Servo angle=============================================
double theta1=0,theta2=0;
//--------------------------------------------------------

//Voltage=================================================
double voltage=16.0;
double voltage_old=16.0;
//--------------------------------------------------------

//boolean=================================================
bool isKill = false; //ASDF
bool isArm = false;  //ASDF
bool isHover = false; //ASDF
bool isHovering = false; //ASDF
bool isLanding = false; //ASDF
bool isTilt = false;
int Hover_Checker = 0;
int Land_Checker = 0;
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
void batteryCallback(const std_msgs::Int16& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void rotCallback(const geometry_msgs::Quaternion& msg);
void filterCallback(const sensor_msgs::Imu& msg);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void setCM();
void publisherSet();
int32_t pwmMapping(double pwm);
void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4, double pwm5, double pwm6, double pwm7, double pwm8);
void pwm_Kill();
void pwm_Max();
void pwm_Arm(); //ASDF
void pwm_test();
void pwm_Calibration();
void kalman_Filtering();
bool GUI_Arm_Callback(FAC_MAV::ArmService::Request &req, FAC_MAV::ArmService::Response &res); // for Arm ASDF 
bool GUI_Kill_Callback(FAC_MAV::KillService::Request &req, FAC_MAV::KillService::Response &res); // for Kill
bool GUI_PosCtrlService_Callback(FAC_MAV::PosCtrlService::Request &req, FAC_MAV::PosCtrlService::Response &res); // ASDF for Hover, Land, Positon_ctrl
bool GUI_Tilt_Callback(FAC_MAV::TiltService::Request &req, FAC_MAV::TiltService::Response &res); 
bool GUI_Hover_Callback(FAC_MAV::HoverService::Request &req, FAC_MAV::HoverService::Response &res);  
void pid_Gain_Setting();
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
ros::Publisher kalman_angular_vel;
ros::Publisher kalman_angular_accel;
ros::Publisher desired_force;
ros::Publisher battery_voltage;
ros::Publisher real_voltage;
ros::Publisher force_command;
ros::ServiceClient HoverClient;
//----------------------------------------------------

//Control Matrix---------------------------------------
//Eigen::MatrixXd CM(4,8);
Eigen::MatrixXd CM(6,6);
//Eigen::Vector4d u;
Eigen::VectorXd u(6);
Eigen::VectorXd F_cmd(6);
Eigen::MatrixXd invCM(6,6);
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
//GUIClient
//FAC_MAV::FAC_HoverService Service;  //ASDF

//Kalman_Filter----------------------------------------
Eigen::MatrixXd F(6,6);
Eigen::MatrixXd P(6,6);
Eigen::MatrixXd H(3,6);
Eigen::MatrixXd Q(6,6);
Eigen::MatrixXd R(3,3);
Eigen::MatrixXd K(6,3);
Eigen::VectorXd x(6);
Eigen::Vector3d z;
Eigen::MatrixXd predicted_P(6,6);
Eigen::VectorXd predicted_x(6);
Eigen::Matrix3d gain_term;
Eigen::MatrixXd covariance_term;
geometry_msgs::Vector3 filtered_Angular_vel;
geometry_msgs::Vector3 filtered_Angular_accel;
//-----------------------------------------------------
double iterator = 0;

int main(int argc, char **argv){
	
    	ros::init(argc, argv,"FAC_MAV_controller");

    	std::string deviceName;
    	ros::NodeHandle params("~");
    	params.param<std::string>("device", deviceName, "/gx5");

    	ros::NodeHandle nh;

	//Loading gains from the "t3_mav_controller.launch" file
		//integratior(PID) limitation
		integ_limit=nh.param<double>("attitude_integ_limit",10);
		z_integ_limit=nh.param<double>("altitude_integ_limit",100);
		pos_integ_limit=nh.param<double>("position_integ_limit",10);

		//Center of Mass
		x_c=nh.param<double>("x_center_of_mass",0.0);
		y_c=nh.param<double>("y_center_of_mass",0.0);
		z_c=nh.param<double>("z_center_of_mass",0.0);

		//Conventional Flight Mode Control Gains
			//Roll, Pitch PID gains
			conv_Pa=nh.param<double>("conv_attitude_rp_P_gain",3.5);
			conv_Ia=nh.param<double>("conv_attitude_rp_I_gain",0.4);
			conv_Da=nh.param<double>("conv_attitude_rp_D_gain",0.5);

			//Yaw PID gains
			conv_Py=nh.param<double>("conv_attitude_y_P_gain",2.0);
			conv_Dy=nh.param<double>("conv_attitude_y_D_gain",0.1);

			//Altitude PID gains
			conv_Pz=nh.param<double>("conv_altitude_P_gain",16.0);
			conv_Iz=nh.param<double>("conv_altitude_I_gain",5.0);
			conv_Dz=nh.param<double>("conv_altitude_D_gain",15.0);

			//Position PID gains
			conv_Pp=nh.param<double>("conv_position_P_gain",3.0);
			conv_Ip=nh.param<double>("conv_position_I_gain",0.1);
			conv_Dp=nh.param<double>("conv_position_D_gain",5.0);

		//Tilt Flight Mode Control Gains
			//Roll, Pitch PID gains
			tilt_Pa=nh.param<double>("tilt_attitude_rp_P_gain",3.5);
			tilt_Ia=nh.param<double>("tilt_attitude_rp_I_gain",0.4);
			tilt_Da=nh.param<double>("tilt_attitude_rp_D_gain",0.5);

			//Yaw PID gains
			tilt_Py=nh.param<double>("tilt_attitude_y_P_gain",5.0);
			tilt_Dy=nh.param<double>("tilt_attitude_y_D_gain",0.3);

			//Altitude PID gains
			tilt_Pz=nh.param<double>("tilt_altitude_P_gain",15.0);
			tilt_Iz=nh.param<double>("tilt_altitude_I_gain",5.0);
			tilt_Dz=nh.param<double>("tilt_altitude_D_gain",10.0);

			//Position PID gains
			tilt_Pp=nh.param<double>("tilt_position_P_gain",3.0);
			tilt_Ip=nh.param<double>("tilt_position_I_gain",0.1);
			tilt_Dp=nh.param<double>("tilt_position_D_gain",3.0);
	//----------------------------------------------------------
	
	//Set Control Matrix----------------------------------------
		setCM();
	//----------------------------------------------------------

	//Kalman initialization-------------------------------------
		x << 0, 0, 0, 0, 0, 0;
		P << Eigen::MatrixXd::Identity(6,6);
		F << Eigen::MatrixXd::Identity(3,3), 0.005*Eigen::MatrixXd::Identity(3,3),
		     Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3);
		H << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);
		Q << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3),
		     Eigen::MatrixXd::Zero(3,3), 100.*Eigen::MatrixXd::Identity(3,3);
		R << 0.0132*Eigen::MatrixXd::Identity(3,3);
        //----------------------------------------------------------
	
    	PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1);
	PWM_generator = nh.advertise<std_msgs::Int32MultiArray>("/command",1);
    	goal_dynamixel_position_  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",100);
	euler = nh.advertise<geometry_msgs::Vector3>("angle",1);
	desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",100);
	desired_altitude = nh.advertise<std_msgs::Float32>("desired_altitude",100);
	Forces = nh.advertise<geometry_msgs::Quaternion>("Forces",100);
	desired_torque = nh.advertise<geometry_msgs::Vector3>("torque_d",100);
	i_result = nh.advertise<geometry_msgs::Vector3>("i_result",100);
	linear_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel",100);
	angular_velocity = nh.advertise<geometry_msgs::Vector3>("gyro",100);
	desired_position = nh.advertise<geometry_msgs::Vector3>("pos_d",100);
	position = nh.advertise<geometry_msgs::Vector3>("pos",100);
	desired_force = nh.advertise<geometry_msgs::Vector3>("force_d",100);
	kalman_angular_vel = nh.advertise<geometry_msgs::Vector3>("kalman_ang_vel",100);
	kalman_angular_accel = nh.advertise<geometry_msgs::Vector3>("kalman_ang_accel",100);
	battery_voltage = nh.advertise<std_msgs::Float32>("battery_voltage",100);
	real_voltage = nh.advertise<std_msgs::Float32>("real_voltage",100);
	force_command = nh.advertise<std_msgs::Float32MultiArray>("force_cmd",100);


    	ros::Subscriber dynamixel_state = nh.subscribe("joint_states",100,jointstateCallback,ros::TransportHints().tcpNoDelay());
    	ros::Subscriber att = nh.subscribe("/gx5/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay());
    	ros::Subscriber rc_in = nh.subscribe("/sbus",100,sbusCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber battery_checker = nh.subscribe("/battery",100,batteryCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_pos=nh.subscribe("/t265_pos",100,posCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_rot=nh.subscribe("/t265_rot",100,rotCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_odom=nh.subscribe("/rs_t265/odom/sample",100,t265OdomCallback,ros::TransportHints().tcpNoDelay());
	
        //GUI GroundStation----------------------------------------
	ros::ServiceServer ArmService = nh.advertiseService("/ArmService", GUI_Arm_Callback); //ASDF
	ros::ServiceServer KillService = nh.advertiseService("/KillService", GUI_Kill_Callback); //ASDF
	ros::ServiceServer PosCtrlService = nh.advertiseService("/PosCtrlService", GUI_PosCtrlService_Callback); //ASDF
	ros::ServiceServer HoverService = nh.advertiseService("/HoverService", GUI_Hover_Callback); //ASDF
    	//HoverClient = nh.serviceClient<FAC_MAV::FAC_HoverService>("/FAC_HoverService"); //ASDF
	ros::ServiceServer TiltService = nh.advertiseService("/TiltService", GUI_Tilt_Callback);

	ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    	ros::spin();
    	return 0;
}

ros::Time loop_timer;

void publisherSet(){
    	//Publish data
		
	
	if(!isTilt){
		theta1_command=0.0;
		theta2_command=0.0;
	}

	if(isKill || Sbus[4]<1500){	
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
		if(isArm){ //AAAA
			if(isHover) rpyT_ctrl();
			else pwm_Arm();
		}
		else{
			pwm_Kill();
		}
		// ROS_INFO("r:%lf, p:%lf, y:%lf T:%lf", r_d, p_d, y_d, T_d);
		//rpyT_ctrl();	
	
	}
	
	//pwm_Calibration();
	//std::cout << H <<std::endl;
	kalman_Filtering();	
	angle_d.x=r_d;
	angle_d.y=p_d;
	angle_d.z=y_d;
	desired_pos.x = X_d;
	desired_pos.y = Y_d;
	desired_pos.z = z_d;
	altitude_d.data=z_d;
	battery_voltage_msg.data=voltage;
	force_cmd.data.resize(6);
	for(int i=0;i<6;i++){
		force_cmd.data[i]=F_cmd(i);
	}
	PWMs.publish(PWMs_cmd);
	euler.publish(imu_rpy);
	desired_angle.publish(angle_d);
	Forces.publish(Force);
	desired_altitude.publish(altitude_d);
	goal_dynamixel_position_.publish(servo_msg_create(theta1_command,-theta2_command));
	desired_torque.publish(torque_d);
	i_result.publish(integrator);
	linear_velocity.publish(lin_vel);
	PWM_generator.publish(PWMs_val);
	desired_position.publish(desired_pos);
	position.publish(pos);
	desired_force.publish(force_d);
	kalman_angular_vel.publish(filtered_Angular_vel);
	kalman_angular_accel.publish(filtered_Angular_accel);
	battery_voltage.publish(battery_voltage_msg);
	real_voltage.publish(battery_real_voltage);
	force_command.publish(force_cmd);
	// ROS_INFO("%d %d %d %d",PWMs_cmd.data[0],PWMs_cmd.data[1],PWMs_cmd.data[2],PWMs_cmd.data[3]);
	//ROS_INFO("loop time : %f",(((double)ros::Time::now().sec-(double)loop_timer.sec)+((double)ros::Time::now().nsec-(double)loop_timer.nsec)/1000000000.));
	loop_timer = ros::Time::now();
}

void setCM(){
	//Counter-rotating type
	/*CM << y_c+(l_servo+z_c)*theta1, l_arm+y_c, y_c+(l_servo+z_c)*theta1, y_c-l_arm, y_c+(l_servo+z_c)*theta1, l_arm+y_c, y_c+(l_servo+z_c)*theta1, y_c-l_arm,
	      l_arm-x_c, -x_c-(l_servo-z_c)*theta2, -l_arm-x_c, -x_c-(l_servo-z_c)*theta2, l_arm-x_c, -x_c-(l_servo-z_c)*theta2, -l_arm-x_c, -x_c-(l_servo-z_c)*theta2,
	      -b_over_k_ratio+(l_arm-x_c)*theta1, b_over_k_ratio-(l_arm+y_c)*theta2, -b_over_k_ratio-(l_arm+x_c)*theta1, b_over_k_ratio+(l_arm-y_c)*theta2, b_over_k_ratio+(l_arm-x_c)*theta1, -b_over_k_ratio-(l_arm+y_c)*theta2, b_over_k_ratio-(l_arm+x_c)*theta1, -b_over_k_ratio+(l_arm-y_c)*theta2,
	      -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0;*/
	//Co-rotating type
	CM <<           -y_c,    -(y_c+l_arm),           -y_c,    -(y_c-l_arm), -b_over_k_ratio,    l_servo+z_c,
                -(l_arm-x_c),             x_c,      l_arm+x_c,             x_c,     l_servo-z_c, b_over_k_ratio,
              b_over_k_ratio, -b_over_k_ratio, b_over_k_ratio, -b_over_k_ratio,             y_c,           -x_c,
                           0,               0,              0,               0,             1.0,              0,
                           0,               0,              0,               0,               0,            1.0,
                         1.0,             1.0,            1.0,             1.0,               0,              0;
       	invCM = CM.inverse();
	//invCM = CM.completeOrthogonalDecomposition().pseudoInverse();
	F_cmd << 0, 0, 0, 0, 0, 0;
}
void rpyT_ctrl() {

	pid_Gain_Setting();	
	if(isLanding)
	{
		if(z_d>-0.1) z_d -= Landing_Inc; // 2.5초간 하강 ASDF
		else
		{
			if(pos.z<0.15)
			{
				Land_Checker ++;

					if(Land_Checker>=100) //0.5초간 유지 ASDF
					{
						isLanding = false;
						isHover = false; //ASDF
						isArm = false; //ASDF
						//Service.request.FAC_isLanding = false;
						//Service.request.FAC_isHover = false;
						//Service.request.FAC_isHovering = false;
						//HoverClient.call(Service);
					}
			}
			else Land_Checker = 0;		
		}
	}

	if(isHovering){
		if(z_d<=1) z_d += Hovering_Inc; // 2.5초간 상승 ASDF
		else
		{
			//if(pos.z>0.9 && pos.z<1.1)  //1미터 +- 10cm 범위내에 도달하면
			//{
			//Hover_Checker ++;

				//if(Hover_Checker >= 600)
				//{
					isHovering = false;
					//Service.request.FAC_isHovering = false;
					//Service.request.FAC_isHover = true;
					//Service.request.FAC_isLanding = false;
					//HoverClient.call(Service);
					z_Goal = z_d;
				//}
			//}
			//else Hover_Checker = 0;
		
		}
	}

	double e_r = 0;
	double e_p = 0;
	double e_y = 0;
	double e_X = 0;
	double e_Y = 0;
	double e_z = z_d - pos.z;
	
	if (!isHovering || !isLanding )
	{
		if(X_Goal - X_d >= X_Inc ) X_d +=X_Inc;
		if(X_Goal - X_d <= -X_Inc ) X_d -=X_Inc;

		if(Y_Goal - Y_d >= Y_Inc ) Y_d +=Y_Inc;
		if(Y_Goal - Y_d <= -Y_Inc) Y_d -=Y_Inc;   

		if(y_Goal - y_d >= yaw_Inc ) y_d +=yaw_Inc; //ASDF
		if(y_Goal - y_d <= -yaw_Inc) y_d -=yaw_Inc;

		if(z_Goal - z_d >= z_Inc ) z_d +=z_Inc;
		if(z_Goal - z_d <= -z_Inc) z_d -=z_Inc;
	}
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

	if(isTilt){
		r_d = 0.0;
		p_d = 0.0;
	}
	else{
		theta1_command = 0.0;
		theta2_command = 0.0;
		r_d = asin(alpha);
		p_d = asin(beta/cos(imu_rpy.x));
		if(fabs(r_d)>rp_limit) r_d = (r_d/fabs(r_d))*rp_limit;
		if(fabs(p_d)>rp_limit) p_d = (p_d/fabs(p_d))*rp_limit;
		//ROS_INFO("Position Control!!");
	}
	
	e_r = r_d - imu_rpy.x;
	e_p = p_d - imu_rpy.y;
	e_y = y_d - imu_rpy.z;

	
	e_r_i += e_r * ((double)1 / freq);
	if (fabs(e_r_i) > integ_limit)	e_r_i = (e_r_i / fabs(e_r_i)) * integ_limit;
	e_p_i += e_p * ((double)1 / freq);
	if (fabs(e_p_i) > integ_limit)	e_p_i = (e_p_i / fabs(e_p_i)) * integ_limit;
	e_z_i += e_z * ((double)1 / freq);	
	if (fabs(e_z_i) > z_integ_limit) e_z_i = (e_z_i / fabs(e_z_i)) * z_integ_limit;
	integrator.x=e_r_i;
	integrator.y=e_p_i;
	integrator.z=e_z_i;

	tau_r_d = Pa * e_r + Ia * e_r_i + Da * (-imu_ang_vel.x);//- (double)0.48;
	tau_p_d = Pa * e_p + Ia * e_p_i + Da * (-imu_ang_vel.y);//+ (double)0.18; 

	if(true){ //AAAA Sbus[5]>1500 ASDF
		Thrust_d =-(Pz * e_z + Iz * e_z_i - Dz * (-v(2)) + mass*g);
		// ROS_INFO("Altitude Control!!");
	}
	else{
		Thrust_d=T_d;
		// ROS_INFO("Manual Thrust!!");
	}
	if(Thrust_d > -0.5*mass*g) Thrust_d = -0.5*mass*g;
	if(Thrust_d < -1.5*mass*g) Thrust_d = -1.5*mass*g;

	Z_ddot_d = Thrust_d/mass+g;
	F_xd = mass*(X_ddot_d*cos(imu_rpy.z)*cos(imu_rpy.y)+Y_ddot_d*sin(imu_rpy.z)*cos(imu_rpy.y)-(Z_ddot_d-g)*sin(imu_rpy.y));
	F_yd = mass*(-X_ddot_d*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+Y_ddot_d*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d-g)*cos(imu_rpy.y)*sin(imu_rpy.x));
	F_zd = mass*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d-g)*cos(imu_rpy.x)*cos(imu_rpy.y));
	if(F_zd > -0.5*mass*g) F_zd = -0.5*mass*g;
	if(F_zd < -1.5*mass*g) F_zd = -1.5*mass*g;
	if(fabs(F_xd) > fabs(F_zd/2.0)*tan(servo_command_limit)) F_xd = F_xd/fabs(F_xd)*(fabs(F_zd/2.0)*tan(servo_command_limit));
	if(fabs(F_yd) > fabs(F_zd/2.0)*tan(servo_command_limit)) F_yd = F_yd/fabs(F_yd)*(fabs(F_zd/2.0)*tan(servo_command_limit));

	double tau_y_d = Py * e_y + Dy * (-imu_ang_vel.z);
	if(fabs(tau_y_d) > tau_y_limit) tau_y_d = tau_y_d/fabs(tau_y_d)*tau_y_limit;

	u << tau_r_d, tau_p_d, tau_y_d, F_xd, F_yd, F_zd;
	torque_d.x = tau_r_d;
	torque_d.y = tau_p_d;
	torque_d.z = tau_y_d;
	force_d.x = F_xd;
	force_d.y = F_yd;
	force_d.z = F_zd;
	//ROS_INFO("xvel:%lf, yvel:%lf, zvel:%lf", imu_ang_vel.x, imu_ang_vel.y, imu_ang_vel.z);
	// ROS_INFO("tr:%lf, tp:%lf, ty:%lf, Thrust_d:%lf", tau_r_d, tau_p_d, tau_y_d, Thrust_d);
	// ROS_INFO("%f",Dz*freq*delta_z);
	ud_to_PWMs(tau_r_d, tau_p_d, tau_y_d, Thrust_d);
}

 

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des) {
	
	/*//Conventional type
	F1 = +((double)0.25 / l_arm) * tau_p_des - ((double)0.125 / b_over_k_ratio) * tau_y_des - (double)0.125 * Thrust_des;
	F2 = +((double)0.25 / l_arm) * tau_r_des + ((double)0.125 / b_over_k_ratio) * tau_y_des - (double)0.125 * Thrust_des;
	F3 = -((double)0.25 / l_arm) * tau_p_des - ((double)0.125 / b_over_k_ratio) * tau_y_des - (double)0.125 * Thrust_des;
	F4 = -((double)0.25 / l_arm) * tau_r_des + ((double)0.125 / b_over_k_ratio) * tau_y_des - (double)0.125 * Thrust_des;
 	F5 = +((double)0.25 / l_arm) * tau_p_des + ((double)0.125 / b_over_k_ratio) * tau_y_des - (double)0.125 * Thrust_des;
	F6 = +((double)0.25 / l_arm) * tau_r_des - ((double)0.125 / b_over_k_ratio) * tau_y_des - (double)0.125 * Thrust_des;
	F7 = -((double)0.25 / l_arm) * tau_p_des + ((double)0.125 / b_over_k_ratio) * tau_y_des - (double)0.125 * Thrust_des;
	F8 = -((double)0.25 / l_arm) * tau_r_des - ((double)0.125 / b_over_k_ratio) * tau_y_des - (double)0.125 * Thrust_des;
 	*/
	//Tilting type
    	F_cmd=invCM*u;

	if(!isTilt){
		F1 = +((double)0.5 / l_arm) * tau_p_des - ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
		F2 = +((double)0.5 / l_arm) * tau_r_des + ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
		F3 = -((double)0.5 / l_arm) * tau_p_des - ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
		F4 = -((double)0.5 / l_arm) * tau_r_des + ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
		//ROS_INFO("%lf %lf %lf %lf",F1,F2,F3,F4);
	}
	//Tilting type
	else {
		
		// F_cmd = [F_(1,z) F_(2,z) F_(3,z) F_(4,z) F_x F_y F_xg F_yg]
		//ROS_INFO("%lf",-F_cmd(5));
		theta1_command = atan2(F_cmd(5),fabs(F_cmd(0)+F_cmd(2)));
		theta2_command = atan2(-F_cmd(4),fabs(F_cmd(1)+F_cmd(3)));
 		if(fabs(theta1_command)>hardware_servo_limit) theta1_command = (theta1_command/fabs(theta1_command))*hardware_servo_limit;
		if(fabs(theta2_command)>hardware_servo_limit) theta2_command = (theta2_command/fabs(theta2_command))*hardware_servo_limit;
		F1 = sqrt(pow(F_cmd(5)/(double)2.0,2)+pow(F_cmd(0),2));
		F2 = sqrt(pow(F_cmd(4)/(double)2.0,2)+pow(F_cmd(1),2));
		F3 = sqrt(pow(F_cmd(5)/(double)2.0,2)+pow(F_cmd(2),2));
		F4 = sqrt(pow(F_cmd(4)/(double)2.0,2)+pow(F_cmd(3),2));
		//F1 = fabs(F_cmd(0));
		//F2 = fabs(F_cmd(1));
		//F3 = fabs(F_cmd(2));
		//F4 = fabs(F_cmd(3));
	}
	pwm_Command(Force_to_PWM(F1),Force_to_PWM(F2), Force_to_PWM(F3), Force_to_PWM(F4), Force_to_PWM(F1), Force_to_PWM(F2), Force_to_PWM(F3), Force_to_PWM(F4));
	Force.x = F1;
	Force.y = F2;
	Force.z = F3;
	Force.w = F4;
	// ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
	// ROS_INFO("%f 1:%d, 2:%d, 3:%d, 4:%d",z_d,PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
}

 

double Force_to_PWM(double F) {
	double pwm;
	double param1 = 610;
	double param2 = 100000;
	double param3 = 230730;
	//Force=1E-05*PWM^2-0.0096*PWM-0.5279
	//PWM=610+(100000*Force+230730)^0.5
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

void batteryCallback(const std_msgs::Int16& msg){
	int16_t value=msg.data;
	voltage=value*3.3/(double)4096/(7440./(30000.+7440.));
	battery_real_voltage.data = voltage;
	double kv=0.08;
	voltage=kv*voltage+(1-kv)*voltage_old;
	voltage_old=voltage;
	if(voltage>16.8) voltage=16.8;
	if(voltage<13.0) voltage=13.0;
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
// ASDF
bool GUI_Arm_Callback(FAC_MAV::ArmService::Request &req, FAC_MAV::ArmService::Response &res){
	if(req.Arm_isChecked){
		isArm = true;
	}
	else{
		isArm = false;
	}
	return true;
}

bool GUI_Kill_Callback(FAC_MAV::KillService::Request &req, FAC_MAV::KillService::Response &res){
	if(req.Kill_isChecked){
		isKill = true;
	}
	else{
		isKill = false;
	}
	return true;
}

bool GUI_Tilt_Callback(FAC_MAV::TiltService::Request &req, FAC_MAV::TiltService::Response &res){
	if(req.Tilt_isChecked){
		isTilt = true;
	}
	else{
		isTilt = false;
	}
	return true;
}

bool GUI_PosCtrlService_Callback(FAC_MAV::PosCtrlService::Request &req, FAC_MAV::PosCtrlService::Response &res){  //ASDF
	X_Goal = req.desired_X;
	Y_Goal = req.desired_Y;
	y_Goal = req.desired_Yaw;
	z_Goal = req.desired_Alti;
	
	return true;
}

bool GUI_Hover_Callback(FAC_MAV::HoverService::Request &req, FAC_MAV::HoverService::Response &res)  //ASDF
{
	isHover = req.isHover;
	isHovering = req.isHovering;
	isLanding = req.isLanding;
	
	return true;
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

void kalman_Filtering(){
	z << imu_ang_vel.x, imu_ang_vel.y, imu_ang_vel.z;
	predicted_x = F*x;
	predicted_P = F*P*F.transpose()+Q;
	gain_term = H*predicted_P*H.transpose()+R;
	K=predicted_P*H.transpose()*gain_term.inverse();
	x=predicted_x+K*(z-H*predicted_x);
	covariance_term = Eigen::MatrixXd::Identity(6,6)-K*H;
	P=covariance_term*predicted_P*covariance_term.transpose()+K*R*K.transpose();
	filtered_Angular_vel.x=x(0);
	filtered_Angular_vel.y=x(1);
	filtered_Angular_vel.z=x(2);
	filtered_Angular_accel.x=x(3);
	filtered_Angular_accel.y=x(4);
	filtered_Angular_accel.z=x(5);
}

void pid_Gain_Setting(){
	if(!isTilt){
		Pa = conv_Pa;
	       	Ia = conv_Ia;
		Da = conv_Da;

		Py = conv_Py;
		Dy = conv_Dy;

		Pz = conv_Pz;
		Iz = conv_Iz;
		Dz = conv_Dz;

		Pp = conv_Pp;
		Ip = conv_Ip;
		Dp = conv_Dp;
		//ROS_INFO("conv");
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

		Pp = tilt_Pp;
		Ip = tilt_Ip;
		Dp = tilt_Dp;
		//ROS_INFO("tilt");
	}
	//ROS_INFO("%.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf ",Pa, Ia, Da, Py, Dy, Pz, Iz, Dz, Pp, Ip, Dp);
}

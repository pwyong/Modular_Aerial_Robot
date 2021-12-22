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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

double freq=200;//controller loop frequency

uint16_t Sbus[8];
int16_t PWM_d;
uint16_t loop_time;
std_msgs::Int16MultiArray PWMs_cmd;
geometry_msgs::Quaternion Force; //Only for plotting in Python (Hard to use Float32MultiArray)

sensor_msgs::JointState rac_servo_value;
sensor_msgs::Imu imu;
geometry_msgs::Quaternion imu_quaternion;
geometry_msgs::Vector3 imu_rpy;
geometry_msgs::Vector3 imu_ang_vel;
geometry_msgs::Vector3 imu_lin_acc;
geometry_msgs::Vector3 angle_d;
geometry_msgs::Vector3 pos;
geometry_msgs::Quaternion rot;
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
double e_z_i = 0;//altitude error integration

double tau_r_d = 0;//roll  desired torque (N.m)
double tau_p_d = 0;//pitch desired torque(N.m)

double Thrust_d = 0;//altitiude desired thrust(N)
double prev_z=0;
//ud_cmd

double r_d = 0;//desired roll angle
double p_d = 0;//desired pitch angle
double y_d = 0;//desired yaw angle
double y_d_tangent = 0;//yaw increment tangent
double T_d = 0;//desired thrust
double z_d = 0;//desired altitude

//--------------------------------------------------------

//General dimensions

static double l_arm = 0.109;// m // diagonal length between thruster : 218mm;
static double l_servo = 0.045;
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
double servo_limit=0.3;
//--------------------------------------------------------

//Control gains===========================================

//integratior(PID) limitation
double integ_limit=2;
double z_integ_limit=100;

//Roll, Pitch PID gains
double Pa=3.0;
double Ia=0.01;
double Da=0.4;


//Yaw PID gains
double Py=0.7;
double Dy=0.1;

//Altitude PID gains
double Pz=10.0;
double Iz=3.0;
double Dz=4.0;
//--------------------------------------------------------

//Servo angle=============================================
double theta1=0,theta2=0;
//--------------------------------------------------------


template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void rpyT_ctrl(double roll_d, double pitch_d, double yaw_d, double altitude_d);
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
void setCM();

ros::Publisher PWMs;
ros::Publisher goal_dynamixel_position_;
ros::Publisher euler;
ros::Publisher desired_angle;
ros::Publisher desired_altitude;
ros::Publisher Forces;

Eigen::Matrix4d CM;
Eigen::Matrix4d invCM;
Eigen::Vector4d u;
Eigen::Vector4d F;


int main(int argc, char **argv){
	
    ros::init(argc, argv,"t3_mav_controller");

    std::string deviceName;
    ros::NodeHandle params("~");
    params.param<std::string>("device", deviceName, "/gx5");

    ros::NodeHandle nh;

    PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1000);
    goal_dynamixel_position_  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",1000);
	euler = nh.advertise<geometry_msgs::Vector3>("angle",1000);
	desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",1000);
	desired_altitude = nh.advertise<std_msgs::Float32>("desired_altitude",1000);
	Forces = nh.advertise<geometry_msgs::Quaternion>("Forces",1000);

    ros::Subscriber dynamixel_state = nh.subscribe("joint_states",1000,jointstateCallback);
    ros::Subscriber att = nh.subscribe("/"+deviceName+"/imu/data",1000,imu_Callback);
    ros::Subscriber rc_in = nh.subscribe("/sbus",1000,sbusCallback);
	ros::Subscriber loop_timer = nh.subscribe("/loop",1000,loopCallback);
	ros::Subscriber t265_pos=nh.subscribe("/t265_pos",1000,posCallback);
	ros::Subscriber t265_rot=nh.subscribe("/t265_rot",1000,rotCallback);
	
	ros::Time begin;
	ros::Time end;

    ros::Rate loop_rate(200);

    while(ros::ok()){
		begin = ros::Time::now();
		// ROS_INFO("%f",CM(2,1));  
    	//Publish data
		
		if(Sbus[6]>1500){
			theta2_command=servo_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			theta1_command=servo_limit*(((double)Sbus[3]-(double)1500)/(double)500);
			r_d=0.0;
			p_d=0.0;
		}
		else{
			r_d=rp_limit*(((double)Sbus[3]-(double)1500)/(double)500);
			p_d=rp_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			theta1_command=0.0;
			theta2_command=0.0;
		}
		y_d_tangent=y_vel_limit*(((double)Sbus[0]-(double)1500)/(double)500);
		if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
		y_d+=y_d_tangent;
		
		z_d=altitude_limit*(((double)Sbus[2]-(double)1500)/(double)500)+altitude_limit;
		T_d =-T_limit*(((double)Sbus[2]-(double)1500)/(double)500)-T_limit;

		if(Sbus[4]<1500){
			PWMs_cmd.data.resize(4);
			PWMs_cmd.data[0] = 1000;
			PWMs_cmd.data[1] = 1000;
			PWMs_cmd.data[2] = 1000;
			PWMs_cmd.data[3] = 1000;
			y_d=imu_rpy.z;	//[J]This line ensures that yaw desired right after disabling the kill switch becomes current yaw attitude
			initial_z=pos.z;
			e_r_i = 0;
			e_p_i = 0;
			e_z_i = 0;
			start_flag=false;
		}
		else{
			// ROS_INFO("r:%lf, p:%lf, y:%lf T:%lf", r_d, p_d, y_d, T_d);
			if(Sbus[5]>1500){
				rpyT_ctrl(r_d, p_d, y_d, z_d);
			}
			else{
				rpyT_ctrl(r_d, p_d, y_d, T_d);	
			}
			
			
			// PWMs_cmd.data.resize(4);
			// PWMs_cmd.data[0] = 1000;
			// PWMs_cmd.data[1] = 1000;
			// PWMs_cmd.data[2] = 1000;
			// PWMs_cmd.data[3] = 1000;
			
			
		}

		setCM();
		angle_d.x=r_d;
		angle_d.y=p_d;
		angle_d.z=y_d;
		altitude_d.data=z_d;
		PWMs.publish(PWMs_cmd);
		euler.publish(imu_rpy);
		desired_angle.publish(angle_d);
		Forces.publish(Force);
		desired_altitude.publish(altitude_d);
		goal_dynamixel_position_.publish(servo_msg_create(theta2_command,theta1_command));
        // ROS_INFO("%d %d %d %d",PWMs_cmd.data[0],PWMs_cmd.data[1],PWMs_cmd.data[2],PWMs_cmd.data[3]);
		
		// std::cout<<F.transpose()<< "\n" <<std::endl;
		ros::spinOnce();
		loop_rate.sleep();
        end = ros::Time::now();
		// ROS_INFO("loop time : %f",((double)end.sec-(double)begin.sec)+((double)end.nsec-(double)begin.nsec)/1000000000.);
    }
    return 0;
}

void setCM(){
	CM << l_servo*theta1, l_arm, l_servo*theta1, -l_arm,
	      l_arm, -l_servo*theta2, -l_arm, -l_servo*theta2,
		  -b_over_k_ratio+l_arm*theta1, b_over_k_ratio-l_arm*theta2, -b_over_k_ratio-l_arm*theta1, b_over_k_ratio+l_arm*theta2,
		  -1.0, -1.0, -1.0, -1.0;
	invCM=CM.inverse();
}

void rpyT_ctrl(double roll_d, double pitch_d, double yaw_d, double altitude_d) {

	double e_r = roll_d - imu_rpy.x;
	double e_p = pitch_d - imu_rpy.y;
	double e_y = yaw_d - imu_rpy.z;
	double e_z = altitude_d - pos.z;
	double delta_z=pos.z-prev_z;
	

	e_r_i += e_r * ((double)1 / freq);
	if (fabs(e_r_i) > integ_limit)	e_r_i = (e_r_i / fabs(e_r_i)) * integ_limit;
	e_p_i += e_p * ((double)1 / freq);
	if (fabs(e_p_i) > integ_limit)	e_p_i = (e_p_i / fabs(e_p_i)) * integ_limit;
	e_z_i += e_z * ((double)1 / freq);
	if (fabs(e_z_i) > z_integ_limit) e_z_i = (e_z_i / fabs(e_z_i)) * z_integ_limit;


	tau_r_d = Pa * e_r + Ia * e_r_i + Da * (-imu_ang_vel.x) - (double)0.4;
	tau_p_d = Pa * e_p + Ia * e_p_i + Da * (-imu_ang_vel.y) + (double)0.2; 
	
	if(Sbus[5]>1500){
		Thrust_d =-(Pz * e_z + Iz * e_z_i - Dz * delta_z*freq+mass*g);
		// ROS_INFO("Altitude Control!!");
	}
	else{
		Thrust_d=altitude_d;
		// ROS_INFO("Manual Thrust!!");
	}

	double tau_y_d = Py * e_y + Dy * (-imu_ang_vel.z);

	u << tau_r_d, tau_p_d, tau_y_d, Thrust_d;

	//ROS_INFO("xvel:%lf, yvel:%lf, zvel:%lf", imu_ang_vel.x, imu_ang_vel.y, imu_ang_vel.z);
	// ROS_INFO("tr:%lf, tp:%lf, ty:%lf, Thrust_d:%lf", tau_r_d, tau_p_d, tau_y_d, Thrust_d);
	// ROS_INFO("%f",Dz*freq*delta_z);
	ud_to_PWMs(tau_r_d, tau_p_d, tau_y_d, Thrust_d);
	prev_z=pos.z;
}

 

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des) {

	F1 = +((double)0.5 / l_arm) * tau_p_des - ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
	F2 = +((double)0.5 / l_arm) * tau_r_des + ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
	F3 = -((double)0.5 / l_arm) * tau_p_des - ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;
	F4 = -((double)0.5 / l_arm) * tau_r_des + ((double)0.25 / b_over_k_ratio) * tau_y_des - (double)0.25 * Thrust_des;

    F=invCM*u;
	// ROS_INFO("F1:%lf, F2:%lf, F3:%lf, F4:%lf", F1, F2, F3, F4);/
	PWMs_cmd.data.resize(4);
	// PWMs_cmd.data[0] = 1000;
	// PWMs_cmd.data[1] = 1000;
	// PWMs_cmd.data[2] = 1000;
	// PWMs_cmd.data[3] = 1000;
	
	PWMs_cmd.data[0] = Force_to_PWM(F1);
	PWMs_cmd.data[1] = Force_to_PWM(F2);
	PWMs_cmd.data[2] = Force_to_PWM(F3);
	PWMs_cmd.data[3] = Force_to_PWM(F4);
	Force.x=PWMs_cmd.data[0];
	Force.y=PWMs_cmd.data[1];
	Force.z=PWMs_cmd.data[2];
	Force.w=PWMs_cmd.data[3];

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
	if (pwm > 1880)	pwm = 1880;
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
	theta1=msg.position[1];
	theta2=msg.position[0];
    // ROS_INFO("theta1:%lf   theta2:%lf",theta1, theta2);
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
	for(int i=0;i<8;i++){
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
	pos.z=msg.z;

	// ROS_INFO("Translation - [x: %f  y:%f  z:%f]",pos.x,pos.y,pos.z);
	// ROS_INFO("posCallback time : %f",(((double)ros::Time::now().sec-(double)posTimer.sec)+((double)ros::Time::now().nsec-(double)posTimer.nsec)/1000000000.));
	posTimer = ros::Time::now();
}

void rotCallback(const geometry_msgs::Quaternion& msg){
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;

	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);

	tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
	// ROS_INFO("Rotation - [r: %f  p: %f  y:%f]",t265_att.x,t265_att.y,t265_att.z);
}


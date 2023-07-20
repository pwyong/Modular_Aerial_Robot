//2022.05.16 Coaxial-Octorotor version
//2022.06.23 Ground Station Application
//2022.08.XX DOB (Disturbance Observer) Application
//2022.09.05 ESC (Extremum Seeking Control) Application
//2022.09.21 Controller mode selection Application

#include "FAC_MAV_ctrler.hpp"
#include <casadi/casadi.hpp>
#include "DoubleLinkedList.h"
#include <vector>
#include <algorithm>
using namespace casadi;

// MHE
int N_MHE=10;
int n_states=18;
int n_controls=6;
int n_outputs=12;
//SX MHE_X = SX::sym("X",n_states);  //{x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw, omega_x, omega_y, omega_z, F_ex, F_ey, F_ez, tau_ex, tau_ey, tau_ez}
//SX MHE_U = SX::sym("U",n_controls); //{F_x, F_y, F_z, tau_x, tau_y, tau_z}
//SX MHE_X_dot = SX::sym("X_dot",n_states); 
//SX MHE_Y = SX::sym("Y",n_outputs);
DM MHE_control_inputs = DM::zeros(n_controls,N_MHE);
DM MHE_measurements = DM::zeros(n_outputs,N_MHE+1);
double Jxx=0.005; double Jyy=0.005; double Jzz=0.01;

MX f(const MX& x, const MX& u);
MX h(const MX& x);

Slice all_elem;
DM MHE_X_star=DM::zeros(n_states,N_MHE+1);

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
//Function-----------------------------------------------
void publisherSet();
void setCM();
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
void pid_Gain_Setting();
void disturbance_Observer();
void sine_wave_vibration();
void ESC_controller();
void state_Reader();
//void MHE_model_setting();
void MHE_nlp_setting();
void MHE_external_force_estimation();
void get_MHE_measurement_control_input();
//-------------------------------------------------------

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

		//Center of Mass
		x_c_init=nh.param<double>("x_center_of_mass",0.0);
		y_c_init=nh.param<double>("y_center_of_mass",0.0);
		z_c_init=nh.param<double>("z_center_of_mass",0.0);
		
		CoM.x = x_c_init;
		CoM.y = y_c_init;
		CoM.z = z_c_init;
		x_c_hat = x_c_init;
		y_c_hat = y_c_init;
		z_c_hat = z_c_init;
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

			//Velocity PID gains
			conv_Pv=nh.param<double>("conv_velocity_P_gain",5.0);
			conv_Iv=nh.param<double>("conv_velocity_I_gain",1.0);
			conv_Dv=nh.param<double>("conv_velocity_D_gain",5.0);

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

			//Velocity PID gains
			tilt_Pv=nh.param<double>("tilt_velocity_P_gain",5.0);
			tilt_Iv=nh.param<double>("tilt_velocity_I_gain",0.1);
			tilt_Dv=nh.param<double>("tilt_velocity_D_gain",5.0);

			//Position PID gains
			tilt_Pp=nh.param<double>("tilt_position_P_gain",3.0);
			tilt_Ip=nh.param<double>("tilt_position_I_gain",0.1);
			tilt_Dp=nh.param<double>("tilt_position_D_gain",5.0);

	//----------------------------------------------------------
	//MHE setting-----------------------------------------------
//		MHE_nlp_setting();
	//----------------------------------------------------------
	//Set Control Matrix----------------------------------------
		setCM();
        F_cmd << 0, 0, 0, 0;
	//----------------------------------------------------------

    PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1); // PWM 1,2,3,4
	PWM_generator = nh.advertise<std_msgs::Int32MultiArray>("/command",1);  // publish to pca9685
    goal_dynamixel_position_  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",100); // desired theta1,2
	euler = nh.advertise<geometry_msgs::Vector3>("angle",1); // roll, pitch, yaw
	desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",100);
	Forces = nh.advertise<std_msgs::Float32MultiArray>("Forces",100); // F 1,2,3,4
	desired_torque = nh.advertise<geometry_msgs::Vector3>("torque_d",100);
	linear_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel",100);
	angular_velocity = nh.advertise<geometry_msgs::Vector3>("angular_velocity",100);
	desired_position = nh.advertise<geometry_msgs::Vector3>("pos_d",100);
	position = nh.advertise<geometry_msgs::Vector3>("pos",100);
	desired_force = nh.advertise<geometry_msgs::Vector3>("force_d",100);
	battery_voltage = nh.advertise<std_msgs::Float32>("battery_voltage",100);
	force_command = nh.advertise<std_msgs::Float32MultiArray>("force_cmd",100);
	delta_time = nh.advertise<std_msgs::Float32>("delta_t",100);
	desired_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel_d",100);
	Center_of_Mass = nh.advertise<geometry_msgs::Vector3>("Center_of_Mass",100);
	angular_Acceleration = nh.advertise<geometry_msgs::Vector3>("ang_accel",100);
	sine_wave_data = nh.advertise<geometry_msgs::Vector3>("sine_wave",100);
	disturbance = nh.advertise<geometry_msgs::Vector3>("att_dhat",100);
	linear_acceleration = nh.advertise<geometry_msgs::Vector3>("lin_acl",100);
	bias_gradient = nh.advertise<geometry_msgs::Vector3>("bias_gradient",100);
	filtered_bias_gradient = nh.advertise<geometry_msgs::Vector3>("filtered_bias_gradient",1);

    ros::Subscriber dynamixel_state = nh.subscribe("joint_states",100,jointstateCallback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber att = nh.subscribe("/gx5/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber rc_in = nh.subscribe("/sbus",100,sbusCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber battery_checker = nh.subscribe("/battery",100,batteryCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_pos=nh.subscribe("/t265_pos",100,posCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_rot=nh.subscribe("/t265_rot",100,rotCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_odom=nh.subscribe("/rs_t265/odom/sample",100,t265OdomCallback,ros::TransportHints().tcpNoDelay());
	
	ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    ros::spin();
    return 0;
}

void publisherSet(){

	end=std::chrono::high_resolution_clock::now();
	delta_t=end-start; 
	dt.data=delta_t.count();
	start=std::chrono::high_resolution_clock::now();
	// F << Eigen::MatrixXd::Identity(3,3), delta_t.count()*Eigen::MatrixXd::Identity(3,3),
	// 	     Eigen::MatrixXd::Zero(3,3),                 Eigen::MatrixXd::Identity(3,3);
//	state_Reader();
	MHE_external_force_estimation();
	get_MHE_measurement_control_input();
	if(!position_mode){
		X_d_base=pos.x;
		Y_d_base=pos.y;
		X_d = X_d_base;
		Y_d = Y_d_base;
		e_X_i=0;
		e_Y_i=0;
		if(attitude_mode){
			e_X_dot_i=0;
			e_Y_dot_i=0;	
		}	
	}

	if(kill_mode){	
		y_d=imu_rpy.z;	//[J]This line ensures that yaw desired right after disabling the kill switch becomes current yaw attitude
		Z_d_base=pos.z;
		e_r_i = 0;
		e_p_i = 0;
		e_Z_i = 0;
		e_Z_dot_i=0;
		e_X_i=0;
		e_X_dot_i=0;
		e_Y_i=0;
		e_Y_dot_i=0;
		start_flag=false;
		theta1_command=0.0;
		theta2_command=0.0;
		pwm_Kill();	
	}
	else{
		
		rpyT_ctrl();		
	}
	
	//pwm_Calibration();	
	angle_d.x=r_d;
	angle_d.y=p_d;
	angle_d.z=y_d;
	desired_pos.x = X_d;
	desired_pos.y = Y_d;
	desired_pos.z = Z_d;
	Force.data.resize(4);
	Force.data[0] = F1;
	Force.data[1] = F2;
	Force.data[2] = F3;
	Force.data[3] = F4;
	CoM.x = x_c_hat;
	CoM.y = y_c_hat;
	CoM.z = z_c_hat;
	dhat.x = dhat_r;
	dhat.y = dhat_p;
	PWMs.publish(PWMs_cmd);
	euler.publish(imu_rpy);
	desired_angle.publish(angle_d);
	Forces.publish(Force);
	goal_dynamixel_position_.publish(servo_msg_create(theta1_command,-theta2_command));
	desired_torque.publish(torque_d);
	linear_velocity.publish(lin_vel);
	PWM_generator.publish(PWMs_val);
	desired_position.publish(desired_pos);
	position.publish(pos);
	desired_force.publish(force_d);
	battery_voltage.publish(battery_voltage_msg);
	force_command.publish(force_cmd);
	delta_time.publish(dt);
	desired_velocity.publish(desired_lin_vel);
	Center_of_Mass.publish(CoM);
	angular_velocity.publish(imu_ang_vel);
	angular_Acceleration.publish(angular_Accel);
	sine_wave_data.publish(sine_wave);
	disturbance.publish(dhat);
	linear_acceleration.publish(lin_acl);
	bias_gradient.publish(bias_gradient_data);
	filtered_bias_gradient.publish(filtered_bias_gradient_data);
	prev_angular_Vel = imu_ang_vel;
	prev_lin_vel = lin_vel;
}

void setCM(){
	//Co-rotating type
/*	CM <<          (l_servo+z_c_hat)*sin(theta1)+y_c_hat*cos(theta1),  (r_arm+y_c_hat)*cos(theta2)+b_over_k_ratio*sin(theta2),       (l_servo+z_c_hat)*sin(theta1)+y_c_hat*cos(theta1), -(r_arm-y_c_hat)*cos(theta2)+b_over_k_ratio*sin(theta2),
              (r_arm-x_c_hat)*cos(theta1)+b_over_k_ratio*sin(theta1),      (-l_servo+z_c_hat)*sin(theta2)-x_c_hat*cos(theta2), -(r_arm+x_c_hat)*cos(theta1)+b_over_k_ratio*sin(theta1),      (-l_servo+z_c_hat)*sin(theta2)-x_c_hat*cos(theta2),
              (r_arm-x_c_hat)*sin(theta1)-b_over_k_ratio*cos(theta1), -(r_arm+y_c_hat)*sin(theta2)+b_over_k_ratio*cos(theta2), -(r_arm+x_c_hat)*sin(theta1)-b_over_k_ratio*cos(theta1),  (r_arm-y_c_hat)*sin(theta2)+b_over_k_ratio*cos(theta2),
                                                        -cos(theta1),                                            -cos(theta2),                                            -cos(theta1),                                            -cos(theta2);
    
*/	CM <<          (l_servo+z_c_hat)*sin(theta1)+y_c_hat*cos(theta1),  (r_arm+y_c_hat)*cos(theta2)+b_over_k_ratio*sin(theta2),       (l_servo+z_c_hat)*sin(theta1)+y_c_hat*cos(theta1), -(r_arm-y_c_hat)*cos(theta2)+b_over_k_ratio*sin(theta2),
              (r_arm-x_c_hat)*cos(theta1)+b_over_k_ratio*sin(theta1),      (-l_servo+z_c_hat)*sin(theta2)-x_c_hat*cos(theta2), -(r_arm+x_c_hat)*cos(theta1)+b_over_k_ratio*sin(theta1),      (-l_servo+z_c_hat)*sin(theta2)-x_c_hat*cos(theta2),
              -b_over_k_ratio*cos(theta1), b_over_k_ratio*cos(theta2), -b_over_k_ratio*cos(theta1),  b_over_k_ratio*cos(theta2),
                                                        -cos(theta1),                                            -cos(theta2),                                            -cos(theta1),                                            -cos(theta2);
	invCM = CM.inverse();
}

void rpyT_ctrl() {
	setCM();
	pid_Gain_Setting();
	y_d_tangent=y_vel_limit*(((double)Sbus[0]-(double)1500)/(double)500);
	if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
	y_d+=y_d_tangent;
	
	Z_d = -altitude_limit*(((double)Sbus[2]-(double)1500)/(double)500)-altitude_limit;
	T_d = -T_limit*(((double)Sbus[2]-(double)1500)/(double)500)-T_limit;

	double e_r = 0;
	double e_p = 0;
	double e_y = 0;
	double e_X = 0;
	double e_Y = 0;
	double e_Z = 0;
	double e_X_dot = 0;
	double e_Y_dot = 0;
		
	double global_X_ddot = (lin_vel.x - prev_lin_vel.x)/delta_t.count();
	double global_Y_ddot = (lin_vel.y - prev_lin_vel.y)/delta_t.count();
	x_ax_dot=-accel_cutoff_freq*x_ax+global_X_ddot;
	x_ax+=x_ax_dot*delta_t.count();
	lin_acl.x=accel_cutoff_freq*x_ax;
	x_ay_dot=-accel_cutoff_freq*x_ay+global_Y_ddot;
	x_ay+=x_ay_dot*delta_t.count();
	lin_acl.y=accel_cutoff_freq*x_ay;
	//ROS_INFO("%lf",time_count);
	angular_Accel.x = (imu_ang_vel.x-prev_angular_Vel.x)/delta_t.count();
	angular_Accel.y = (imu_ang_vel.y-prev_angular_Vel.y)/delta_t.count();
	angular_Accel.z = (imu_ang_vel.z-prev_angular_Vel.z)/delta_t.count();

	if(position_mode || velocity_mode){
		if(position_mode){
		/*	if(!estimating){
				X_d = X_d_base - XY_limit*(((double)Sbus[1]-(double)1500)/(double)500);
				Y_d = Y_d_base + XY_limit*(((double)Sbus[3]-(double)1500)/(double)500);
				traj_timer = 0;
			}*/
		//	else{
				if(trajectory_flag){
					X_d = X_d_base+Amp-Amp*cos(2*PI/Tp*traj_timer);
					Y_d = Y_d_base+Amp*sin(2*PI/Tp*traj_timer);
					traj_timer+=delta_t.count();
				}
				else{
					if(traj_timer>0.1){
						X_d_base=X_d;
						Y_d_base=Y_d;
						traj_timer = 0;
					}
					X_d = X_d_base - XY_limit*(((double)Sbus[1]-(double)1500)/(double)500);
					Y_d = Y_d_base + XY_limit*(((double)Sbus[3]-(double)1500)/(double)500);
				}
		//	}
			e_X = X_d - pos.x;
			e_Y = Y_d - pos.y;
			e_X_i += e_X * delta_t.count();
			if (fabs(e_X_i) > pos_integ_limit) e_X_i = (e_X_i / fabs(e_X_i)) * pos_integ_limit;
			e_Y_i += e_Y * delta_t.count();
			if (fabs(e_Y_i) > pos_integ_limit) e_Y_i = (e_Y_i / fabs(e_Y_i)) * pos_integ_limit;
	
			X_dot_d = Pp * e_X + Ip * e_X_i - Dp * lin_vel.x;
			Y_dot_d = Pp * e_Y + Ip * e_Y_i - Dp * lin_vel.y;
		}
		if(velocity_mode){
			X_dot_d = -XYZ_dot_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			Y_dot_d = XYZ_dot_limit*(((double)Sbus[3]-(double)1500)/(double)500);
		}	
		if(fabs(X_dot_d) > XYZ_dot_limit) X_dot_d = (X_dot_d/fabs(X_dot_d))*XYZ_dot_limit;
		if(fabs(Y_dot_d) > XYZ_dot_limit) Y_dot_d = (Y_dot_d/fabs(Y_dot_d))*XYZ_dot_limit;
		//*/
		desired_lin_vel.x = X_dot_d;
		desired_lin_vel.y = Y_dot_d;
	
		e_X_dot = X_dot_d - lin_vel.x;
		e_Y_dot = Y_dot_d - lin_vel.y;
		e_X_dot_i += e_X_dot * delta_t.count();
		if (fabs(e_X_dot_i) > vel_integ_limit) e_X_dot_i = (e_X_dot_i / fabs(e_X_dot_i)) * vel_integ_limit;
		e_Y_dot_i += e_Y_dot * delta_t.count();
		if (fabs(e_Y_dot_i) > vel_integ_limit) e_Y_dot_i = (e_Y_dot_i / fabs(e_Y_dot_i)) * vel_integ_limit;

		X_ddot_d = Pv * e_X_dot + Iv * e_X_dot_i - Dv * lin_acl.x;
		Y_ddot_d = Pv * e_Y_dot + Iv * e_Y_dot_i - Dv * lin_acl.y;
		if(fabs(X_ddot_d) > XYZ_ddot_limit) X_ddot_d = (X_ddot_d/fabs(X_ddot_d))*XYZ_ddot_limit;
		if(fabs(Y_ddot_d) > XYZ_ddot_limit) Y_ddot_d = (Y_ddot_d/fabs(Y_ddot_d))*XYZ_ddot_limit;
		
		if(tilt_mode){
			r_d = 0.0;
			p_d = 0.0;
			F_xd = mass*(X_ddot_d*cos(imu_rpy.z)*cos(imu_rpy.y)+Y_ddot_d*sin(imu_rpy.z)*cos(imu_rpy.y)-(Z_ddot_d-g)*sin(imu_rpy.y));
			F_yd = mass*(-X_ddot_d*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+Y_ddot_d*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d-g)*cos(imu_rpy.y)*sin(imu_rpy.x));
			//if(position_mode) ROS_INFO("Position & Tilt !!!");
			//else ROS_INFO("Velocity & Tilt !!!");
		}
		else{
			alpha=(-sin(imu_rpy.z)*X_ddot_d+cos(imu_rpy.z)*Y_ddot_d)/g;
			beta=(-cos(imu_rpy.z)*X_ddot_d-sin(imu_rpy.z)*Y_ddot_d)/g;
			if(fabs(alpha) > alpha_beta_limit) alpha = (alpha/fabs(alpha))*alpha_beta_limit;
			if(fabs(beta) > alpha_beta_limit) beta = (beta/fabs(beta))*alpha_beta_limit;
			r_d = asin(alpha);
			p_d = asin(beta/cos(imu_rpy.x));
			if(fabs(r_d)>rp_limit) r_d = (r_d/fabs(r_d))*rp_limit;
			if(fabs(p_d)>rp_limit) p_d = (p_d/fabs(p_d))*rp_limit;
			
			//if(position_mode) ROS_INFO("Position & Conventional !!!");
			//else ROS_INFO("Velocity & Conventional !!!");
		}
	}
	if(attitude_mode){
		if(tilt_mode){
			r_d = 0.0;
			p_d = 0.0;
			F_xd=-mass*XYZ_ddot_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			F_yd=mass*XYZ_ddot_limit*(((double)Sbus[3]-(double)1500)/(double)500);
			//ROS_INFO("Attitude & Tilt !!!");

		}
		else{
			r_d=rp_limit*(((double)Sbus[3]-(double)1500)/(double)500);
			p_d=rp_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			//ROS_INFO("Attidue Control!!");
			F_xd=0.0;
			F_yd=0.0;
			//ROS_INFO("Attitude & Conventional");
		}
	}
	
	e_r = r_d - imu_rpy.x;
	e_p = p_d - imu_rpy.y;
	e_y = y_d - imu_rpy.z;

	e_Z = Z_d - pos.z;
	
	e_r_i += e_r * delta_t.count();
	if (fabs(e_r_i) > integ_limit)	e_r_i = (e_r_i / fabs(e_r_i)) * integ_limit;
	e_p_i += e_p * delta_t.count();
	if (fabs(e_p_i) > integ_limit)	e_p_i = (e_p_i / fabs(e_p_i)) * integ_limit;
	e_Z_i += e_Z * delta_t.count();	
	if (fabs(e_Z_i) > z_integ_limit) e_Z_i = (e_Z_i / fabs(e_Z_i)) * z_integ_limit;

	tau_r_d = Pa * e_r + Ia * e_r_i + Da * (-imu_ang_vel.x);//- (double)0.48;
	tau_p_d = Pa * e_p + Ia * e_p_i + Da * (-imu_ang_vel.y);//+ (double)0.18; 
	tau_y_d = Py * e_y + Dy * (-imu_ang_vel.z);
	if(fabs(tau_y_d) > tau_y_limit) tau_y_d = tau_y_d/fabs(tau_y_d)*tau_y_limit;
	
	if(altitude_mode){
		Z_ddot_d = Pz * e_Z + Iz * e_Z_i - Dz * lin_vel.z;
		desired_lin_vel.z = Z_ddot_d; // But this is desired acceleration
		if(Sbus[6]>1500) F_zd = mass*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d-g)*cos(imu_rpy.x)*cos(imu_rpy.y));
		else F_zd = mass*(Z_ddot_d-g);
		//ROS_INFO("Altitude");
	}
	else{
		e_Z_i = 0;
		e_Z_dot_i = 0;
		F_zd=T_d;
		//ROS_INFO("Manual Thrust!!");
	}
	if(F_zd > -0.5*mass*g) F_zd = -0.5*mass*g;
	if(F_zd < -2.0*mass*g) F_zd = -2.0*mass*g;
	
	//ESC-----------------------------------------------------
	//if(ESC_control){
	//	ESC_controller();
	//}
	//--------------------------------------------------------

	//DOB-----------------------------------------------------
	if(DOB_mode){
		disturbance_Observer();
	}
	else{
	    	x_r1 = 0.0; 
		x_r2 = 0.0; 
		x_r3 = 0.0;
		y_r1 = 0.0;
		y_r2 = 0.0;
		y_r3 = 0.0;
		x_p1 = 0.0;
		x_p2 = 0.0;
		x_p3 = 0.0;
		y_p1 = 0.0;
		y_p2 = 0.0;
		y_p3 = 0.0;
		dhat_r = 0.0;
		dhat_p = 0.0; 
	}
	//--------------------------------------------------------
	tautilde_r_d = tau_r_d - dhat_r;
	tautilde_p_d = tau_p_d - dhat_p;
	//u << tau_r_d, tau_p_d, tau_y_d, F_zd;
	u << tautilde_r_d, tautilde_p_d, tau_y_d, F_zd;
	torque_d.x = tautilde_r_d;
	torque_d.y = tautilde_p_d;
	torque_d.z = tau_y_d;
	force_d.x = F_xd;
	force_d.y = F_yd;
	force_d.z = F_zd;

	prev_angular_Vel = imu_ang_vel;
	ud_to_PWMs(tau_r_d, tau_p_d, tau_y_d, Thrust_d);
	//ud_to_PWMs(tautilde_r_d, tautilde_p_d, tautilde_y_d, Thrust_d);
}

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des) {	
 	
	//Co-rotating coaxial
	//Conventional type
	F_cmd = invCM*u;
	if(!tilt_mode){
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
    	if(base_yaw - yaw_prev < -PI) yaw_rotate_count++;
	else if(base_yaw - yaw_prev > PI) yaw_rotate_count--;
	yaw_now = base_yaw+2*PI*yaw_rotate_count;
	//ROS_INFO("now : %lf / prev : %lf / count : %d",yaw_now, yaw_prev, yaw_rotate_count);
	imu_rpy.z = yaw_now;
	yaw_prev = base_yaw;
	// ROS_INFO("imuCallback time : %f",(((double)ros::Time::now().sec-(double)imuTimer.sec)+((double)ros::Time::now().nsec-(double)imuTimer.nsec)/1000000000.));
	//imuTimer = ros::Time::now();
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
		
	if(Sbus[9]>1500) DOB_mode=true;
	else DOB_mode=false;
	
	
//	if(Sbus[9]>1500) trajectory_flag=true;
//	else trajectory_flag=false;
//	*/
	
}


void batteryCallback(const std_msgs::Int16& msg){
	int16_t value=msg.data;
	voltage=value*3.3/(double)4096/(7300./(30000.+7300.));
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
	R_v << cos(PI/2.), -sin(PI/2.),  0.,
 	       sin(PI/2.),  cos(PI/2.),  0.,
	                0.,           0.,  1.;

	v = R_v*cam_v;

	double global_X_dot = v(2)*(sin(imu_rpy.x)*sin(imu_rpy.z-PI/4.0)+cos(imu_rpy.x)*cos(imu_rpy.z-PI/4.0)*sin(imu_rpy.y))-v(1)*(cos(imu_rpy.x)*sin(imu_rpy.z-PI/4.0)-cos(imu_rpy.z-PI/4.0)*sin(imu_rpy.x)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.z-PI/4.0)*cos(imu_rpy.y);
	double global_Y_dot = v(1)*(cos(imu_rpy.x)*cos(imu_rpy.z-PI/4.0)+sin(imu_rpy.x)*sin(imu_rpy.z-PI/4.0)*sin(imu_rpy.y))-v(2)*(cos(imu_rpy.z-PI/4.0)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.z-PI/4.0)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.y)*sin(imu_rpy.z-PI/4.0);
	double global_Z_dot = -v(0)*sin(imu_rpy.y)+v(2)*cos(imu_rpy.x)*cos(imu_rpy.y)+v(1)*cos(imu_rpy.y)*sin(imu_rpy.x);

	lin_vel.x=global_X_dot;
	lin_vel.y=global_Y_dot;
	lin_vel.z=global_Z_dot;
	//ROS_INFO("Attitude - [r: %f  p: %f  y:%f]",cam_att(0),cam_att(1),cam_att(2));
	//ROS_INFO("Linear_velocity - [x: %f  y: %f  z:%f]",v(0),v(1),v(2));
	//ROS_INFO("Angular_velocity - [x: %f  y: %f  z:%f]",w(0),w(1),w(2));
}


void pid_Gain_Setting(){
	if(!tilt_mode){
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
//	dhat.x = dhat_r;
//	dhat.y = dhat_p;
//	dhat.z = dhat_y;

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

void ESC_controller(){
		sine_wave_vibration();
		
			
			F_zd = F_zd + vibration1;
			
			x_dot_11 = -pass_freq1/Q_factor*x_11-pow(pass_freq1,2.0)*x_12+MoI_y_hat*angular_Accel.y;
			x_dot_12 = x_11;
			x_11 += x_dot_11*delta_t.count();
			x_12 += x_dot_12*delta_t.count();
			y_11 = pass_freq1/Q_factor*x_11;
			double gradient_bias_x_c = vibration1*y_11;
			x_grad_x_dot=-xy_cutoff_freq*x_grad_x+gradient_bias_x_c;
			x_grad_x+=x_grad_x_dot*delta_t.count();
			filtered_grad_x=xy_cutoff_freq*x_grad_x;
			bias_x_c += filtered_grad_x*delta_t.count();
			x_c_hat = x_c_init-G_XY*bias_x_c;
			if(fabs(x_c_hat)>x_c_limit) x_c_hat = x_c_hat/fabs(x_c_hat)*x_c_limit;
		
			x_dot_21 = -pass_freq1/Q_factor*x_21-pow(pass_freq1,2.0)*x_22+MoI_x_hat*angular_Accel.x;
			x_dot_22 = x_21;
			x_21 += x_dot_21*delta_t.count();
			x_22 += x_dot_22*delta_t.count();
			y_21 = pass_freq1/Q_factor*x_21;
			double gradient_bias_y_c =vibration1*y_21;
			x_grad_y_dot=-xy_cutoff_freq*x_grad_y+gradient_bias_y_c;
			x_grad_y+=x_grad_y_dot*delta_t.count();
			filtered_grad_y=xy_cutoff_freq*x_grad_y;
			bias_y_c += filtered_grad_y*delta_t.count();
			y_c_hat = y_c_init+G_XY*bias_y_c;
			if(fabs(y_c_hat)>y_c_limit) y_c_hat = y_c_hat/fabs(y_c_hat)*y_c_limit;
		
		//	*/
		//if(!z_c_convergence){
			//F_xd = F_xd + vibration2;
			F_yd = F_yd + vibration2;
		
			x_dot_31 = -pass_freq2/Q_factor*x_31-pow(pass_freq2,2.0)*x_32+MoI_x_hat*angular_Accel.x;
			x_dot_32 = x_31;
			x_31 += x_dot_31*delta_t.count();
			x_32 += x_dot_32*delta_t.count();
			y_31 = pass_freq2/Q_factor*x_31;
			double gradient_bias_z_c = vibration2*y_31;
			x_grad_z_dot=-z_cutoff_freq*x_grad_z+gradient_bias_z_c;
			x_grad_z+=x_grad_z_dot*delta_t.count();
			filtered_grad_z=z_cutoff_freq*x_grad_z;
			bias_z_c += filtered_grad_z*delta_t.count();
			z_c_hat = z_c_init-G_Z*bias_z_c;
			if(fabs(z_c_hat)>z_c_limit) z_c_hat = z_c_hat/fabs(z_c_hat)*z_c_limit;
	 	//}

		bias_gradient_data.x=gradient_bias_x_c;
		bias_gradient_data.y=gradient_bias_y_c;
		bias_gradient_data.z=gradient_bias_z_c;

		filtered_bias_gradient_data.x=filtered_grad_x;
		filtered_bias_gradient_data.y=filtered_grad_y;
		filtered_bias_gradient_data.z=filtered_grad_z;
		//ROS_INFO("ESC");
}

void state_Reader(){
	if(!hovering){
		if(Z_d<-0.3){
			if(fabs(Z_d-pos.z)<0.02){
				hovering_time_count+=delta_t.count();
				hovering_force+=F_zd;
				hovering_count++;
				if(hovering_time_count>3.0) {
					hovering=true;
					hovering_force=hovering_force/hovering_count;
					ROS_INFO("Hovering");
					ROS_INFO("Hovering force : %lf",hovering_force);
				}
			}
			else{
				hovering_time_count = 0;
				hovering_count = 0;
				hovering_force = 0;
			}
		}
	}

	if(!loading){
	 	if(hovering){
	 		if(/*F_zd-hovering_force<-2.0 &&*/ fabs(Z_d-pos.z)<0.02 && fabs(r_d-imu_rpy.x)<0.05 && fabs(p_d-imu_rpy.y)<0.05){
	 			loading_time_count+=delta_t.count();
				loading_force+=F_zd;
				loading_count++;
	 			if(loading_time_count>2.0){
	 				loading = true;
	 				loading_force=loading_force/loading_count;
					//double F_gap = hovering_force-loading_force;
					//double F_gapmax = hovering_force -(-1.5*mass*g); 
	 				ROS_INFO("Loading");
					ROS_INFO("Loading force : %lf",loading_force);
					ROS_INFO("Estimating CoM...");
					tilt_mode = true;
					ESC_control = true;
					//DOB_mode=true;
					
	 			}
	 		}
			else{
				loading_time_count=0;
				loading_count=0;
				loading_force=0;
			}
	 	}
	}
	else{
		if(hovering){
			if(estimation_timer > 60.0){
				if(!x_c_convergence && fabs(filtered_grad_x*1000)<0.1){
					x_c_convergence_time_count+=delta_t.count();
					if(x_c_convergence_time_count>5.0){
						x_c_convergence = true;
						ROS_INFO("x_c Estimation complete");
					}
				}
				else{
			//		if(x_c_convergence && fabs(filtered_grad_x*1000)>0.3){
			//			x_c_convergence = false;
			//			ROS_INFO("x_c yet");
			//		}
					x_c_convergence_time_count = 0;
				}
				if(!y_c_convergence && fabs(filtered_grad_y*1000)<0.1){
					y_c_convergence_time_count+=delta_t.count();
					if(y_c_convergence_time_count>5.0){
						y_c_convergence = true;
						ROS_INFO("y_c Estimation complete");
					}
				}
				else{
			//		if(y_c_convergence && fabs(filtered_grad_y*1000)>0.3){
			//			y_c_convergence = false;
			//			ROS_INFO("y_c yet");
			//		}
					y_c_convergence_time_count= 0;
				}
				if(!z_c_convergence && fabs(filtered_grad_z*1000)<0.05){
					z_c_convergence_time_count+=delta_t.count();
					if(z_c_convergence_time_count>5.0){
						z_c_convergence = true;
						ROS_INFO("z_c Estimation complete");
					}
				}
				else{
			//		if(z_c_convergence && fabs(filtered_grad_z*1000)>0.2){
			//			z_c_convergence = false;
			//			ROS_INFO("z_c yet");
			//		}
					z_c_convergence_time_count = 0;
				}
			}	
			if(x_c_convergence && y_c_convergence && z_c_convergence && !estimating){
				estimating = true;
				ESC_control = false;
				ROS_INFO("CoM Estimation complete");
				ROS_INFO("x_c : %lf / y_c : %lf / z_c : %lf",x_c_hat, y_c_hat, z_c_hat);
				ROS_INFO("Estimation time : %lf",estimation_timer);
				estimation_timer=0;
				tilt_Ia=0.1;
				tilt_Dp=1.0;
			}
			else{
				estimation_timer+=delta_t.count();
			}
		
			if(loading_force-F_zd<-2.0 && fabs(Z_d-pos.z)<0.02){
				unloading_time_count+=delta_t.count();
				if(unloading_time_count>0.5){
					state_init();
					ROS_INFO("UnLoading");
				}
			}
			else{
				unloading_time_count=0;
			}
		}
	}
}

void get_MHE_measurement_control_input(){
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
	if(control_input_F_x.length()==N_MHE) control_input_F_x.deleteHead();
	if(control_input_F_y.length()==N_MHE) control_input_F_y.deleteHead();
	if(control_input_F_z.length()==N_MHE) control_input_F_z.deleteHead();
	if(control_input_tau_x.length()==N_MHE) control_input_tau_x.deleteHead();
	if(control_input_tau_y.length()==N_MHE) control_input_tau_y.deleteHead();
	if(control_input_tau_z.length()==N_MHE) control_input_tau_z.deleteHead();

	measurement_x.insertTail(pos.x);
	measurement_y.insertTail(pos.y);
	measurement_z.insertTail(pos.z);
	measurement_x_dot.insertTail(lin_vel.x);	
	measurement_y_dot.insertTail(lin_vel.y);	
	measurement_z_dot.insertTail(lin_vel.z);
	measurement_roll.insertTail(imu_rpy.x);	
	measurement_pitch.insertTail(imu_rpy.y);	
	measurement_yaw.insertTail(imu_rpy.z);
	measurement_omega_x.insertTail(imu_ang_vel.x);	
	measurement_omega_y.insertTail(imu_ang_vel.y);	
	measurement_omega_z.insertTail(imu_ang_vel.z);
	control_input_F_x.insertTail(force_d.x);	
	control_input_F_y.insertTail(force_d.y);	
	control_input_F_z.insertTail(force_d.z);	
	control_input_tau_x.insertTail(torque_d.x);	
	control_input_tau_y.insertTail(torque_d.y);	
	control_input_tau_z.insertTail(torque_d.z);	
//	std::cout << measurement_x_dot.getData(0) << std::endl;
	for(int i=0;i<N_MHE+1;i++){
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

//	opti.set_value(MHE_U,MHE_control_inputs);
//	opti.set_value(MHE_Y,MHE_measurements);
}

MX f(const MX& x, const MX& u){
	MX dx=MX::zeros(n_states,1);
	dx(0)=x(3);
	dx(1)=x(4);
	dx(2)=x(5);
	dx(3)=(x(12) - u(1) * (cos(x(6)) * sin(x(8)) - cos(x(8)) * sin(x(6)) * sin(x(7))) + u(2) * (sin(x(6)) * sin(x(8)) + cos(x(6)) * cos(x(8)) * sin(x(7))) + u(0) * cos(x(8)) * cos(x(7))) / mass;
	dx(4)=(x(13) + u(1) * (cos(x(6)) * cos(x(8)) + sin(x(6)) * sin(x(8)) * sin(x(7))) - u(2) * (cos(x(8)) * sin(x(6)) - cos(x(6)) * sin(x(8)) * sin(x(7))) + u(0) * cos(x(7)) * sin(x(8))) / mass;
	dx(5)=g + (x(14) - u(0) * sin(x(7)) + u(2) * cos(x(6)) * cos(x(7)) + u(1) * cos(x(7)) * sin(x(6))) / mass;
	dx(6)=x(9) + sin(x(6)) * tan(x(7)) * x(10) + cos(x(6)) * tan(x(7)) * x(11);
	dx(7)=cos(x(6)) * x(10) - sin(x(6)) * x(11);
	dx(8)=sin(x(6)) / cos(x(7)) * x(10) + cos(x(6)) / cos(x(7)) * x(11);
	dx(9)=(u(3) + x(15)) / Jxx;
   	dx(10)=(u(4) + x(16)) / Jyy;
      	dx(11)=(u(5) + x(17)) / Jzz;
	dx(12)=0; dx(13)=0; dx(14)=0;
	dx(15)=0; dx(16)=0; dx(17)=0;
	return x+dx*delta_t.count();
/*	return MX::vertcat({x(0)+x(3)*delta_t.count(),
		       x(1)+x(4)*delta_t.count(),
                       x(2)+x(5)*delta_t.count(),
		       x(3)+((x(12) - u(1) * (cos(x(6)) * sin(x(8)) - cos(x(8)) * sin(x(6)) * sin(x(7))) + u(2) * (sin(x(6)) * sin(x(8)) + cos(x(6)) * cos(x(8)) * sin(x(7))) + u(0) * cos(x(8)) * cos(x(7))) / mass)*delta_t.count(),
		       x(4)+((x(13) + u(1) * (cos(x(6)) * cos(x(8)) + sin(x(6)) * sin(x(8)) * sin(x(7))) - u(2) * (cos(x(8)) * sin(x(6)) - cos(x(6)) * sin(x(8)) * sin(x(7))) + u(0) * cos(x(7)) * sin(x(8))) / mass)*delta_t.count(),
		       x(5)+(g + (x(14) - u(0) * sin(x(7)) + u(2) * cos(x(6)) * cos(x(7)) + u(1) * cos(x(7)) * sin(x(6))) / mass)*delta_t.count(),
		       x(6)+(x(9) + sin(x(6)) * tan(x(7)) * x(10) + cos(x(6)) * tan(x(7)) * x(11))*delta_t.count(),
		       x(7)+(cos(x(6)) * x(10) - sin(x(6)) * x(11))*delta_t.count(),
		       x(8)+(sin(x(6)) / cos(x(7)) * x(10) + cos(x(6)) / cos(x(7)) * x(11))*delta_t.count(),
		       x(9)+((u(3) + x(15)) / Jxx)*delta_t.count(),
		       x(10)+((u(4) + x(16)) / Jyy)*delta_t.count(),
		       x(11)+((u(5) + x(17)) / Jzz)*delta_t.count(),
		       x(12), 
		       x(13), 
		       x(14),
		       x(15), 
		       x(16), 
		       x(17)});*/
}

MX h(const MX& x){
/*	MX y=opti.variable(n_outputs,1);
	y(0)=x(0);	
	y(1)=x(1);	
	y(2)=x(2);	
	y(3)=x(3);	
	y(4)=x(4);	
	y(5)=x(5);	
	y(6)=x(6);	
	y(7)=x(7);	
	y(8)=x(8);	
	y(9)=x(9);	
	y(10)=x(10);	
	y(11)=x(11);	*/
	return x(Slice(0,n_outputs));
}

void MHE_nlp_setting(){
/*
	for(int k=0; k<N_MHE+1; k++){
		MX st=MHE_X(all_elem,k);
		MX y_tilde=MHE_Y(all_elem,k);
		MX h_x=h(st);//MHE_X(Slice(0,n_outputs),k);
		obj+=mtimes(mtimes((y_tilde-h_x).T(),output_stage_cost_weight),(y_tilde-h_x)); 
//		std::cout << h_x.size() << std::endl; 
	//	obj+=test_mx;
		if(k<N_MHE){
			MX con=MHE_U(all_elem,k);
			MX next_st=MHE_X(all_elem,k+1);
			MX f_xu=f(st,con);
			obj+=mtimes(mtimes((next_st-f_xu).T(),state_stage_cost_weight),(next_st-f_xu));
		}
	}
	MX init_st=MHE_X(all_elem,0);
	obj+=mtimes(mtimes((init_st-MHE_init_X_tilde).T(),arrival_cost_weight),(init_st-MHE_init_X_tilde));
	
	opti.minimize(obj);
	
	for(int k=0;k<N_MHE;k++){
		MX st=MHE_X(all_elem,k);
		MX con=MHE_U(all_elem,k);
		MX st_next=f(st,con);
		opti.subject_to(MHE_X(all_elem,k+1)==st_next);	
	}

	opti.subject_to(-5<=MHE_x_pos<=5);
	opti.subject_to(-5<=MHE_y_pos<=5);
	opti.subject_to(-3<=MHE_z_pos<=0.5);
	opti.subject_to(-3<=MHE_x_vel<=3);
	opti.subject_to(-3<=MHE_y_vel<=3);
	opti.subject_to(-5<=MHE_z_vel<=5);
	opti.subject_to(-0.5<=MHE_roll<=0.5);
	opti.subject_to(-0.5<=MHE_pitch<=0.5);
	opti.subject_to(-PI<=MHE_yaw<=PI);
	opti.subject_to(-2<=MHE_omega_x<=2);
	opti.subject_to(-2<=MHE_omega_y<=2);
	opti.subject_to(-2<=MHE_omega_z<=2);
	opti.subject_to(-5<=MHE_F_ex<=5);
	opti.subject_to(-5<=MHE_F_ey<=5);
	opti.subject_to(-5<=MHE_F_ez<=5);
	opti.subject_to(-3<=MHE_tau_ex<=3);
	opti.subject_to(-3<=MHE_tau_ey<=3);
	opti.subject_to(-3<=MHE_tau_ez<=3);
	opti.subject_to(obj>=0);

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
	opti.set_initial(MHE_F_ez,0);
	opti.set_initial(MHE_tau_ex,0);
	opti.set_initial(MHE_tau_ey,0);
	opti.set_initial(MHE_tau_ez,0);

	opti.set_value(MHE_init_X_tilde,MHE_X_star(all_elem,1));	
	opti.set_value(MHE_Y,MHE_measurements);
	opti.set_value(MHE_U,MHE_control_inputs);
	

	Dict solver_option;
	//solver_option["ipopt.max_iter"]=2000;
	solver_option["print_time"]=0;
	solver_option["ipopt.print_level"]=0;
	solver_option["ipopt.acceptable_tol"]=1e-8;
	solver_option["ipopt.acceptable_obj_change_tol"]=1e-6;
		
	opti.solver("ipopt",solver_option);
*/
}

void MHE_external_force_estimation(){
//	std::cout << obj.size() << std::endl;

Opti opti;
MX obj=opti.variable();
MX MHE_X=opti.variable(n_states,N_MHE+1); 
MX MHE_Y=opti.parameter(n_outputs,N_MHE+1);
MX MHE_U=opti.parameter(n_controls,N_MHE);
MX MHE_init_X_tilde=opti.parameter(n_states,1);
MX output_stage_cost_weight=MX::eye(n_outputs)*10;
MX state_stage_cost_weight=MX::eye(n_states);
MX arrival_cost_weight=MX::eye(n_states);

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

	for(int k=0; k<N_MHE+1; k++){
		MX st=MHE_X(all_elem,k);
		MX y_tilde=MHE_Y(all_elem,k);
		MX h_x=h(st);//MHE_X(Slice(0,n_outputs),k);
		obj+=mtimes(mtimes((y_tilde-h_x).T(),output_stage_cost_weight),(y_tilde-h_x)); 
//		std::cout << h_x.size() << std::endl; 
	//	obj+=test_mx;
		if(k<N_MHE){
			MX con=MHE_U(all_elem,k);
			MX next_st=MHE_X(all_elem,k+1);
			MX f_xu=f(st,con);
			obj+=mtimes(mtimes((next_st-f_xu).T(),state_stage_cost_weight),(next_st-f_xu));
		}
	}
	MX init_st=MHE_X(all_elem,0);
	obj+=mtimes(mtimes((init_st-MHE_init_X_tilde).T(),arrival_cost_weight),(init_st-MHE_init_X_tilde));
	
	opti.minimize(obj);
	
	for(int k=0;k<N_MHE;k++){
		MX st=MHE_X(all_elem,k);
		MX con=MHE_U(all_elem,k);
		MX st_next=f(st,con);
		opti.subject_to(MHE_X(all_elem,k+1)==st_next);	
	}

	opti.subject_to(-5<=MHE_x_pos<=5);
	opti.subject_to(-5<=MHE_y_pos<=5);
	opti.subject_to(-3<=MHE_z_pos<=0.5);
	opti.subject_to(-3<=MHE_x_vel<=3);
	opti.subject_to(-3<=MHE_y_vel<=3);
	opti.subject_to(-5<=MHE_z_vel<=5);
	opti.subject_to(-0.5<=MHE_roll<=0.5);
	opti.subject_to(-0.5<=MHE_pitch<=0.5);
	opti.subject_to(-PI<=MHE_yaw<=PI);
	opti.subject_to(-2<=MHE_omega_x<=2);
	opti.subject_to(-2<=MHE_omega_y<=2);
	opti.subject_to(-2<=MHE_omega_z<=2);
	opti.subject_to(-5<=MHE_F_ex<=5);
	opti.subject_to(-5<=MHE_F_ey<=5);
	opti.subject_to(-5<=MHE_F_ez<=5);
	opti.subject_to(-3<=MHE_tau_ex<=3);
	opti.subject_to(-3<=MHE_tau_ey<=3);
	opti.subject_to(-3<=MHE_tau_ez<=3);
//	opti.subject_to(obj>=0);

	opti.set_initial(MHE_x_pos,horzcat(MHE_X_star(0,Slice(1,11,1)),MHE_X_star(0,10)));
	opti.set_initial(MHE_y_pos,horzcat(MHE_X_star(1,Slice(1,11,1)),MHE_X_star(1,10)));
	opti.set_initial(MHE_z_pos,horzcat(MHE_X_star(2,Slice(1,11,1)),MHE_X_star(2,10)));
	opti.set_initial(MHE_x_vel,horzcat(MHE_X_star(3,Slice(1,11,1)),MHE_X_star(3,10)));
	opti.set_initial(MHE_y_vel,horzcat(MHE_X_star(4,Slice(1,11,1)),MHE_X_star(4,10)));
	opti.set_initial(MHE_z_vel,horzcat(MHE_X_star(5,Slice(1,11,1)),MHE_X_star(5,10)));
	opti.set_initial(MHE_roll,horzcat(MHE_X_star(6,Slice(1,11,1)),MHE_X_star(6,10)));
	opti.set_initial(MHE_pitch,horzcat(MHE_X_star(7,Slice(1,11,1)),MHE_X_star(7,10)));
	opti.set_initial(MHE_yaw,horzcat(MHE_X_star(8,Slice(1,11,1)),MHE_X_star(8,10)));
	opti.set_initial(MHE_omega_x,horzcat(MHE_X_star(9,Slice(1,11,1)),MHE_X_star(9,10)));
	opti.set_initial(MHE_omega_y,horzcat(MHE_X_star(10,Slice(1,11,1)),MHE_X_star(10,10)));
	opti.set_initial(MHE_omega_z,horzcat(MHE_X_star(11,Slice(1,11,1)),MHE_X_star(11,10)));
	opti.set_initial(MHE_F_ex,horzcat(MHE_X_star(12,Slice(1,11,1)),MHE_X_star(12,10)));
	opti.set_initial(MHE_F_ey,horzcat(MHE_X_star(13,Slice(1,11,1)),MHE_X_star(13,10)));
	opti.set_initial(MHE_F_ez,horzcat(MHE_X_star(14,Slice(1,11,1)),MHE_X_star(14,10)));
	opti.set_initial(MHE_tau_ex,horzcat(MHE_X_star(15,Slice(1,11,1)),MHE_X_star(15,10)));
	opti.set_initial(MHE_tau_ey,horzcat(MHE_X_star(16,Slice(1,11,1)),MHE_X_star(16,10)));
	opti.set_initial(MHE_tau_ez,horzcat(MHE_X_star(17,Slice(1,11,1)),MHE_X_star(17,10)));
	
	Dict solver_option;
	solver_option["ipopt.max_iter"]=2000;
	solver_option["print_time"]=0;
//	solver_option["ipopt.print_level"]=0;
	solver_option["ipopt.acceptable_tol"]=1e-8;
	solver_option["ipopt.acceptable_obj_change_tol"]=1e-6;
		
	opti.solver("ipopt",solver_option);

	opti.set_value(MHE_init_X_tilde,MHE_X_star(all_elem,1));	
	opti.set_value(MHE_Y,MHE_measurements);
	opti.set_value(MHE_U,MHE_control_inputs);

	
	OptiSol sol=opti.solve();	
	MHE_X_star = sol.value(MHE_X);
	//opti.set_value(MHE_init_X_tilde,MHE_X_star(all_elem,1));	
	std::cout << sol.value(MHE_X_star(12,10)) << std::endl;
	std::cout << std::endl;

}

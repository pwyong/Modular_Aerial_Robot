#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <ros/ros.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>

uint16_t Arr[8];
void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array);  

int main(int argc, char **argv){
	ros::init(argc, argv, "arduino_sub");
	ros::NodeHandle nh;
	
	// ros::Publisher PWMs=nh.advertise<std_msgs::Int16MultiArray>("SBUS", 100);
	ros::Subscriber arduino_sub=nh.subscribe("/sbus", 100, &sbusCallback);
	ros::Rate loop_rate(200);
	std_msgs::Int16MultiArray SBUS_temp;

	while(ros::ok()){
		SBUS_temp.data.resize(8);
		for(int i=0;i<8;i++){
			SBUS_temp.data[i]=Arr[i];
		}
		ROS_INFO("SBUS - [%d, %d, %d, %d, %d, %d, %d, %d]",SBUS_temp.data[0], SBUS_temp.data[1], SBUS_temp.data[2], SBUS_temp.data[3], SBUS_temp.data[4], SBUS_temp.data[5], SBUS_temp.data[6], SBUS_temp.data[7]);
			
		// PWMs.publish(PWM_temp);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array){
	for(int i=0;i<8;i++){
		Arr[i]=array->data[i];
	}
	return;
}
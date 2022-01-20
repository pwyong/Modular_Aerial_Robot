#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"tf_listener");

    ros::NodeHandle nh;

    ros::Publisher pos=nh.advertise<geometry_msgs::Vector3>("/t265_pos",100);
    ros::Publisher rot=nh.advertise<geometry_msgs::Quaternion>("/t265_rot",100);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::Buffer tfBuffer2;
    tf2_ros::TransformListener tfListener2(tfBuffer2);

    ros::Rate rate(250);
    while(nh.ok()){
        geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::TransformStamped transformStamped2;
        try{
            transformStamped = tfBuffer.lookupTransform("t4","world",ros::Time(0));
	    transformStamped2 = tfBuffer2.lookupTransform("drone_world","t4",ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Vector3 trans;
        geometry_msgs::Quaternion quat;

        trans.x=transformStamped2.transform.translation.x;
        trans.y=transformStamped2.transform.translation.y;
        trans.z=transformStamped2.transform.translation.z;

        quat.x=transformStamped2.transform.rotation.x;
        quat.y=transformStamped2.transform.rotation.y;
        quat.z=transformStamped2.transform.rotation.z;
        quat.w=transformStamped2.transform.rotation.w;

        pos.publish(trans);
        rot.publish(quat);

        rate.sleep();
    }
    return 0;
}

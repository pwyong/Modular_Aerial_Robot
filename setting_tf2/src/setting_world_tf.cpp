#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#define PI 3.141592

int main(int argc, char **argv){
    ros::init(argc,argv,"setting_world_tf");

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id="world";
    static_transformStamped.child_frame_id="rs_t265_odom_frame";
    static_transformStamped.transform.translation.x=0.08;
    static_transformStamped.transform.translation.y=0.16;
    static_transformStamped.transform.translation.z=0.152;
    tf2::Quaternion quat;
    quat.setRPY(0.0,0.0,-90.0*PI/180.0);
    static_transformStamped.transform.rotation.x=quat.x();
    static_transformStamped.transform.rotation.y=quat.y();
    static_transformStamped.transform.rotation.z=quat.z();
    static_transformStamped.transform.rotation.w=quat.w();

    static_broadcaster.sendTransform(static_transformStamped);
    ros::spin();
    return 0;
}

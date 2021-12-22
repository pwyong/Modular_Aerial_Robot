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

    ros::Rate rate(200);
    while(nh.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("t4","world",ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Vector3 trans;
        geometry_msgs::Quaternion quat;

        trans.x=transformStamped.transform.translation.x;
        trans.y=transformStamped.transform.translation.y;
        trans.z=transformStamped.transform.translation.z;

        quat.x=transformStamped.transform.rotation.x;
        quat.y=transformStamped.transform.rotation.y;
        quat.z=transformStamped.transform.rotation.z;
        quat.w=transformStamped.transform.rotation.w;

        pos.publish(trans);
        rot.publish(quat);

        rate.sleep();
    }
    return 0;
}
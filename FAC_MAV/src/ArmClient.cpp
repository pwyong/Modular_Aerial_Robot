#include "ros/ros.h"
#include "FAC_MAV/ArmService.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ArmService");



   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<FAC_MAV::ArmService>("ArmService");
   FAC_MAV::ArmService Service;
 
 return 0;
}

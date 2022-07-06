#include "ros/ros.h"
#include "FAC_MAV/KillService.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "KillService");



   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<FAC_MAV::KillService>("KillService");
   FAC_MAV::KillService Service;
 

 return 0;
}

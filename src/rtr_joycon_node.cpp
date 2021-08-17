#include <ros/ros.h>
#include <rtr_joycon/rtr_joycon.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtr_joycon_node");
  ros::NodeHandle nh;
  RTRJoycon rjc(nh);

  ros::spin();
  
  return 0;
}

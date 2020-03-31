#include <ros/ros.h>
#include <rtr_joycon/rtr_joycon.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtr_joycon_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
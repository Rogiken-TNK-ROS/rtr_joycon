#include <ros/ros.h>
#include <rtr_joycon/rtr_joycon_state.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtr_joycon");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  RtrJoyconState state(nh);

  while (ros::ok())
  {
    state.WriteString("using config : ");
    state.publish();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
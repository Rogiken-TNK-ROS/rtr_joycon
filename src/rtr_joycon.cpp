#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rtr_joycon/rtr_joycon_state.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtr_joycon");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 10);
  ros::Rate loop_rate(10);

  RtrJoyconState state(nh);

  while (ros::ok())
  {
    state.WriteString("AAA");
    state.publish();
    std_msgs::String msg;
    msg.data = "hello world!";
    ROS_INFO("publish: %s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
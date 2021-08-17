#ifndef RTR_JOYCON_H_
#define RTR_JOYCON_H_

#include <ros/ros.h>
#include <radial_menu_model/model.hpp>
#include <sensor_msgs/JointState.h>

#include <rtr_joycon/tohoku_jog_control.h>

class RTRJoycon
{
public:
  RTRJoycon(ros::NodeHandle& nh);

private:
  void joyCb(const sensor_msgs::JoyConstPtr joy);
  void menuCb(const radial_menu_msgs::StateConstPtr menu);
  void jsCb(const sensor_msgs::JointStateConstPtr js);
  
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Subscriber menu_sub_;
  ros::Subscriber js_sub_;
  radial_menu_model::Model model_;
  sensor_msgs::JointState js_;

  rtr::TohokuJogControl tjc_;
  
};

#endif  // RTR_JOYCON_H_

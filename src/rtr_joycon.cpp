#include <rtr_joycon/rtr_joycon.h>

#include <sensor_msgs/Joy.h>
#include <radial_menu_msgs/State.h>

void RTRJoycon::joyCb(const sensor_msgs::JoyConstPtr joy)
{
  if (!model_.isEnabled()) {
    if(model_.isSelected("Teleop Mode.Tohoku Jog.Slow"))
      tjc_.slowControl(*joy, js_);
    if(model_.isSelected("Teleop Mode.Tohoku Jog.Fast"))
      tjc_.fastControl(*joy, js_);
    if(model_.isSelected("Teleop Mode.Mani Jog.Slow"))
      mjc_.slowControl(*joy, js_);
    if(model_.isSelected("Teleop Mode.Mani Jog.Fast"))
      mjc_.fastControl(*joy, js_);
    // if(model_.isSelected("Teleop Mode.Tohoku EEffector.Slow"))
    //   tic_.slowControl(*joy);
    // if(model_.isSelected("Teleop Mode.Tohoku EEffector.Fast"))
    //   tic_.fastControl(*joy);
  }
  bc_.control(*joy);
}

void RTRJoycon::menuCb(const radial_menu_msgs::StateConstPtr menu)
{
  if (!model_.setState(*menu)) {
    ROS_ERROR("menuCb(): Cannot set state to the model");
    return;
  }
}

void RTRJoycon::jsCb(const sensor_msgs::JointStateConstPtr js)
{
  js_ = *js;
}

RTRJoycon::RTRJoycon(ros::NodeHandle& nh) : bc_(nh), tjc_(nh), mjc_(nh), tic_(nh)
{
  // setup subscriber //
  joy_sub_ = nh.subscribe("joy", 1, &RTRJoycon::joyCb, this);
  menu_sub_ = nh.subscribe("teleop_menu_state", 1, &RTRJoycon::menuCb, this);
  js_sub_ = nh.subscribe("RTRDoubleArmV7/joint_states", 2, &RTRJoycon::jsCb, this);
  while(1) {
    if (!model_.setDescriptionFromParam(nh.resolveName("teleop_menu_description"))) {
      ROS_ERROR_STREAM("Cannot set menu description from the param '"
        << nh.resolveName("teleop_menu_description"));
      ros::Duration(1.0).sleep();
    } else
      break;
  }
}

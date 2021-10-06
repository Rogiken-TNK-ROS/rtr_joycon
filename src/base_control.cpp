#include <rtr_joycon/base_control.h>

namespace rtr
{
BaseControl::BaseControl(ros::NodeHandle& n)
{
  ros::NodeHandle nh("~");
  kc_.init(nh);
  // joint map
  std::string str;
  nh.getParam("Linear/PLUS", str);
  joint_map_["Linear/PLUS"] = str;
  nh.getParam("Angular/PLUS", str);
  joint_map_["Angular/PLUS"] = str;
  nh.getParam("Linear/MINUS", str);
  joint_map_["Linear/MINUS"] = str;
  nh.getParam("Angular/MINUS", str);
  joint_map_["Angular/MINUS"] = str;
  
  nh.getParam("BASE/linear_speed", linear_speed_);
  nh.getParam("BASE/angular_speed", angular_speed_);

  twist_pub_ = n.advertise<geometry_msgs::Twist>("RTRDoubleArmV7/base_controller/cmd_vel", 5);
}

void BaseControl::control(const sensor_msgs::Joy& joy)
{
  double plus_linear = 0;
  double minus_linear = 0;
  double plus_angular = 0;
  double minus_angular = 0;
  for(auto i=joint_map_.begin(); i!=joint_map_.end(); i++)
  {
    int idx;
    try { idx = kc_.map(i->second); }
    catch(std::out_of_range&) { continue; };
    
    if(i->second.substr(0,7) == "buttons")
    {
      if(i->first.substr(0,6) == "Linear") {
        if(i->first.substr(7,4) == "PLUS")
          plus_linear = linear_speed_*joy.buttons[kc_.map(i->second)];
        if(i->first.substr(7,5) == "MINUS")
          minus_linear = -linear_speed_*joy.buttons[kc_.map(i->second)];
      } else if(i->first.substr(0,7) == "Angular") {
        if(i->first.substr(8,4) == "PLUS")
          plus_angular = angular_speed_*joy.buttons[kc_.map(i->second)];
        if(i->first.substr(8,5) == "MINUS")
          minus_angular = -angular_speed_*joy.buttons[kc_.map(i->second)];
      } 
    }
  }
  twist_msg_.linear.x = plus_linear + minus_linear;
  twist_msg_.angular.z = plus_angular + minus_angular;
  twist_pub_.publish(twist_msg_);

}

} // namespace rtr

#ifndef RTR_JOYCON_BASE_CONTROL_H_
#define RTR_JOYCON_BASE_CONTROL_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <rtr_joycon/keymap_config.h>

namespace rtr
{
class BaseControl
{
 public:
  BaseControl(ros::NodeHandle& n);

  void control(const sensor_msgs::Joy& joy);

 private:
  rtr::KeymapConfig kc_;
  std::map<std::string, std::string> joint_map_;

  ros::Publisher twist_pub_;

  geometry_msgs::Twist twist_msg_;

  double linear_speed_;
  double angular_speed_;
};

}


#endif // RTR_JOYCON_BASE_CONTROL_H_

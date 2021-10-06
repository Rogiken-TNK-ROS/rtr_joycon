#ifndef RTR_JOYCON_TOHOKU_JOG_CONTROL_H_
#define RTR_JOYCON_TOHOKU_JOG_CONTROL_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <rtr_joycon/keymap_config.h>

namespace rtr
{
class TohokuJogControl
{
 public:
  TohokuJogControl(ros::NodeHandle& n);
  void slowControl(const sensor_msgs::Joy& joy, const sensor_msgs::JointState& js);
  void fastControl(const sensor_msgs::Joy& joy, const sensor_msgs::JointState& js);

 private:
  void control(const sensor_msgs::Joy& joy, const double& step_rad=0.6);

  rtr::KeymapConfig kc_;
  std::map<std::string, std::string> joint_map_;
  std::map<std::string, double> arm_joint_values_;
  std::map<std::string, double> gripper_joint_values_;

  ros::Publisher arm_pub_;
  ros::Publisher gripper_pub_;

  trajectory_msgs::JointTrajectory arm_msg_;
  trajectory_msgs::JointTrajectory gripper_msg_;

  double axis_threshold_{0.5};
  double slow_step_rad_{0.3};
  double fast_step_rad_{0.9};
};

} // namespace rtr

#endif // RTR_JOYCON_TOHOKU_JOG_CONTROL_H_

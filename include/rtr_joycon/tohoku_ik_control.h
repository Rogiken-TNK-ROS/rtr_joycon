#ifndef RTR_JOYCON_TOHOKU_IK_CONTROL_H_
#define RTR_JOYCON_TOHOKU_IK_CONTROL_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <rtr_joycon/keymap_config.h>

namespace rtr
{
class TohokuIKControl
{
 public:
  TohokuIKControl(ros::NodeHandle& n);

  void slowControl(const sensor_msgs::Joy& joy);
  void fastControl(const sensor_msgs::Joy& joy);
  
 private:
  void control(const sensor_msgs::Joy& joy, const double& xyz_step = 0.01, const double& rpy_step=0.01);
  
  rtr::KeymapConfig kc_;
  std::map<std::string, std::string> keymap_;
  
  moveit::planning_interface::MoveGroupInterfacePtr mi_;
  moveit::planning_interface::PlanningSceneInterfacePtr psi_;

  const std::string group_;

  double axis_threshold_{0.5};
  double slow_step_xyz_{0.01};
  double slow_step_rpy_{0.1};
  double fast_step_xyz_{0.1};
  double fast_step_rpy_{0.05};
  int rpy_mode_button_;
  
};

}

#endif // RTR_JOYCON_TOHOKU_IK_CONTROL_H_

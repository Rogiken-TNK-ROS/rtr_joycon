#include <rtr_joycon/mani_jog_control.h>

namespace rtr
{
ManiJogControl::ManiJogControl(ros::NodeHandle& n)
{
  ros::NodeHandle nh("~");
  kc_.init(nh);
  // joint map
  std::string str;
  nh.getParam("UFRAME/PLUS", str);
  joint_map_["UFRAME/PLUS"] = str;
  nh.getParam("MNP_SWING/PLUS", str);
  joint_map_["MNP_SWING/PLUS"] = str;
  nh.getParam("MANIBOOM/PLUS", str);
  joint_map_["MANIBOOM/PLUS"] = str;
  nh.getParam("MANIARM/PLUS", str);
  joint_map_["MANIARM/PLUS"] = str;
  nh.getParam("MANIELBOW/PLUS", str);
  joint_map_["MANIELBOW/PLUS"] = str;
  nh.getParam("YAWJOINT/PLUS", str);
  joint_map_["YAWJOINT/PLUS"] = str;
  nh.getParam("HANDBASE/PLUS", str);
  joint_map_["HANDBASE/PLUS"] = str;
  nh.getParam("PUSHROD/PLUS", str);
  joint_map_["PUSHROD/PLUS"] = str;
  nh.getParam("UFRAME/MINUS", str);
  joint_map_["UFRAME/MINUS"] = str;
  nh.getParam("MNP_SWING/MINUS", str);
  joint_map_["MNP_SWING/MINUS"] = str;
  nh.getParam("MANIBOOM/MINUS", str);
  joint_map_["MANIBOOM/MINUS"] = str;
  nh.getParam("MANIARM/MINUS", str);
  joint_map_["MANIARM/MINUS"] = str;
  nh.getParam("MANIELBOW/MINUS", str);
  joint_map_["MANIELBOW/MINUS"] = str;
  nh.getParam("YAWJOINT/MINUS", str);
  joint_map_["YAWJOINT/MINUS"] = str;
  nh.getParam("HANDBASE/MINUS", str);
  joint_map_["HANDBASE/MINUS"] = str;
  nh.getParam("PUSHROD/MINUS", str);
  joint_map_["PUSHROD/MINUS"] = str;

  nh.getParam("MANI_JOG/axis_threshold", axis_threshold_);
  nh.getParam("MANI_JOG/slow_step_radian", slow_step_rad_);
  nh.getParam("MANI_JOG/fast_step_radian", fast_step_rad_);
    
  arm_joint_values_["UFRAME"] = 0;
  arm_joint_values_["MNP_SWING"] = 0;
  arm_joint_values_["MANIBOOM"] = 0;
  arm_joint_values_["MANIARM"] = 0;
  arm_joint_values_["MANIELBOW"] = 0;
  arm_joint_values_["YAWJOINT"] = 0;
  arm_joint_values_["HANDBASE"] = 0;
  gripper_joint_values_["PUSHROD"] = 0;

  arm_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("RTRDoubleArmV7/mani_arm_controller/command", 5);
  gripper_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("RTRDoubleArmV7/mani_gripper_controller/command", 5);

  arm_msg_.points.resize(1);
  for(auto i=arm_joint_values_.begin(); i!=arm_joint_values_.end(); i++) {
    arm_msg_.joint_names.emplace_back(i->first);
    arm_msg_.points[0].positions.emplace_back(i->second);
    arm_msg_.points[0].time_from_start = ros::Duration(1.0);
  }
  gripper_msg_.points.resize(1);
  for(auto i=gripper_joint_values_.begin(); i!=gripper_joint_values_.end(); i++) {
    gripper_msg_.joint_names.emplace_back(i->first);
    gripper_msg_.points[0].positions.emplace_back(i->second);
    gripper_msg_.points[0].time_from_start = ros::Duration(1.0);
  }
}

void ManiJogControl::slowControl(const sensor_msgs::Joy& joy, const sensor_msgs::JointState& js)
{
  for(uint i=0; i<js.name.size(); i++) {
    try {
      arm_joint_values_.at(js.name[i]) = js.position[i];
      gripper_joint_values_.at(js.name[i]) = js.position[i];
    }
    catch(std::out_of_range&) { continue; };
  }
  control(joy, slow_step_rad_);
}

void ManiJogControl::fastControl(const sensor_msgs::Joy& joy, const sensor_msgs::JointState& js)
{
  for(uint i=0; i<js.name.size(); i++) {
    try {
      arm_joint_values_.at(js.name[i]) = js.position[i];
      gripper_joint_values_.at(js.name[i]) = js.position[i];
    }
    catch(std::out_of_range&) { continue; };
  }
  control(joy, fast_step_rad_);
}

void ManiJogControl::control(const sensor_msgs::Joy& joy, const double& step_rad)
{
  for(auto i=arm_joint_values_.begin(); i!=arm_joint_values_.end(); i++)
  {
    std::string plus_key;
    std::string minus_key;
    try {
      plus_key = joint_map_.at(i->first+"/PLUS");
      minus_key = joint_map_.at(i->first+"/MINUS");
    } catch(std::out_of_range&) {
      continue;
    }
    double plus_value = 0;
    double minus_value = 0;
    if(plus_key.substr(0,4) == "axes")
    {
      if(i->first == "MANIARM") {
        plus_value = -step_rad * (joy.axes[kc_.map(plus_key)] >= axis_threshold_);
        minus_value = step_rad * (joy.axes[kc_.map(minus_key)] <= -axis_threshold_);
      } else {
        plus_value = step_rad * (joy.axes[kc_.map(plus_key)] >= axis_threshold_);
        minus_value = -step_rad * (joy.axes[kc_.map(minus_key)] <= -axis_threshold_);
      }
    }
    else if(plus_key.substr(0,7) == "buttons")
    {
      plus_value = step_rad * joy.buttons[kc_.map(plus_key)];
      minus_value = -step_rad * joy.buttons[kc_.map(minus_key)];
    }
    else
    {
      plus_value = 0;
      minus_value = 0;
    }
    arm_msg_.points[0].positions[std::distance(arm_joint_values_.begin(), i)] = i->second + plus_value + minus_value;
  }
  for(auto i=gripper_joint_values_.begin(); i!=gripper_joint_values_.end(); i++)
  {
    std::string plus_key;
    std::string minus_key;
    try {
      plus_key = joint_map_.at(i->first+"/PLUS");
      minus_key = joint_map_.at(i->first+"/MINUS");
    } catch(std::out_of_range&) {
      continue;
    }
    double plus_value = 0;
    double minus_value = 0;
    if(plus_key.substr(0,4) == "axes")
    {
      plus_value = 0.1 * (joy.axes[kc_.map(plus_key)] >= axis_threshold_);
      minus_value = -0.1 * (joy.axes[kc_.map(minus_key)] <= -axis_threshold_);
    }
    else if(plus_key.substr(0,7) == "buttons")
    {
      plus_value = 0.1 * joy.buttons[kc_.map(plus_key)];
      minus_value = -0.1 * joy.buttons[kc_.map(minus_key)];
    }
    else
    {
      plus_value = 0;
      minus_value = 0;
    }
    if(plus_value != 0 || minus_value != 0)
      gripper_msg_.points[0].positions[std::distance(gripper_joint_values_.begin(), i)] = i->second + plus_value + minus_value;
  }

  arm_msg_.header.stamp = joy.header.stamp;
  gripper_msg_.header.stamp = joy.header.stamp;
  arm_pub_.publish(arm_msg_);
  gripper_pub_.publish(gripper_msg_);
}

} // namespace

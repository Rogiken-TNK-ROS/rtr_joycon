#include <rtr_joycon/tohoku_jog_control.h>

namespace rtr
{
TohokuJogControl::TohokuJogControl(ros::NodeHandle& n)
{
  ros::NodeHandle nh("~");
  kc_.init(nh);
  // joint map
  std::string str;
  nh.getParam("MFRAME/PLUS", str);
  joint_map_["MFRAME/PLUS"] = str;
  nh.getParam("BLOCK/PLUS", str);
  joint_map_["BLOCK/PLUS"] = str;
  nh.getParam("BOOM/PLUS", str);
  joint_map_["BOOM/PLUS"] = str;
  nh.getParam("ARM/PLUS", str);
  joint_map_["ARM/PLUS"] = str;
  nh.getParam("TOHKU_PITCH/PLUS", str);
  joint_map_["TOHKU_PITCH/PLUS"] = str;
  nh.getParam("TOHKU_ROLL/PLUS", str);
  joint_map_["TOHKU_ROLL/PLUS"] = str;
  nh.getParam("TOHKU_TIP_01/PLUS", str);
  joint_map_["TOHKU_TIP_01/PLUS"] = str;
  nh.getParam("TOHKU_TIP_02/PLUS", str);
  joint_map_["TOHKU_TIP_02/PLUS"] = str;
  nh.getParam("MFRAME/MINUS", str);
  joint_map_["MFRAME/MINUS"] = str;
  nh.getParam("BLOCK/MINUS", str);
  joint_map_["BLOCK/MINUS"] = str;
  nh.getParam("BOOM/MINUS", str);
  joint_map_["BOOM/MINUS"] = str;
  nh.getParam("ARM/MINUS", str);
  joint_map_["ARM/MINUS"] = str;
  nh.getParam("TOHKU_PITCH/MINUS", str);
  joint_map_["TOHKU_PITCH/MINUS"] = str;
  nh.getParam("TOHKU_ROLL/MINUS", str);
  joint_map_["TOHKU_ROLL/MINUS"] = str;
  nh.getParam("TOHKU_TIP_01/MINUS", str);
  joint_map_["TOHKU_TIP_01/MINUS"] = str;
  nh.getParam("TOHKU_TIP_02/MINUS", str);
  joint_map_["TOHKU_TIP_02/MINUS"] = str;
  nh.getParam("TOHOKU_JOG/axis_threshold", axis_threshold_);
  nh.getParam("TOHOKU_JOG/slow_step_radian", slow_step_rad_);
  nh.getParam("TOHOKU_JOG/fast_step_radian", fast_step_rad_);
    
  arm_joint_values_["MFRAME"] = 0;
  arm_joint_values_["BLOCK"] = 0;
  arm_joint_values_["BOOM"] = 0;
  arm_joint_values_["ARM"] = 0;
  arm_joint_values_["TOHKU_PITCH"] = 0;
  arm_joint_values_["TOHKU_ROLL"] = 0;
  gripper_joint_values_["TOHKU_TIP_01"] = 0;
  gripper_joint_values_["TOHKU_TIP_02"] = 0;

  arm_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("RTRDoubleArmV7/tohoku_arm_controller/command", 5);
  gripper_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("RTRDoubleArmV7/tohoku_gripper_controller/command", 5);

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

void TohokuJogControl::slowControl(const sensor_msgs::Joy& joy, const sensor_msgs::JointState& js)
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

void TohokuJogControl::fastControl(const sensor_msgs::Joy& joy, const sensor_msgs::JointState& js)
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

void TohokuJogControl::control(const sensor_msgs::Joy& joy, const double& step_rad)
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
      if(i->first == "ARM") {
        plus_value = -step_rad * (joy.axes[kc_.map(plus_key)] >= axis_threshold_);
        minus_value = step_rad * (joy.axes[kc_.map(minus_key)] < -axis_threshold_);
      } else {
        plus_value = step_rad * (joy.axes[kc_.map(plus_key)] >= axis_threshold_);
        minus_value = -step_rad * (joy.axes[kc_.map(minus_key)] < -axis_threshold_);
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
      plus_value = step_rad * (joy.axes[kc_.map(plus_key)] >= axis_threshold_);
      minus_value = -step_rad * (joy.axes[kc_.map(minus_key)] < -axis_threshold_);
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
    gripper_msg_.points[0].positions[std::distance(gripper_joint_values_.begin(), i)] = i->second + plus_value + minus_value;
  }

  arm_msg_.header.stamp = joy.header.stamp;
  gripper_msg_.header.stamp = joy.header.stamp;
  arm_pub_.publish(arm_msg_);
  gripper_pub_.publish(gripper_msg_);
}

} // namespace

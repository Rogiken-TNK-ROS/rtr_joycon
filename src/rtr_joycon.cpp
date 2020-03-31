#include <rtr_joycon/rtr_joycon.h>

RTRJoycon::RTRJoycon(ros::NodeHandle& nh)
  : nh_(nh),
    speed_rate_(0.2), 
    config_num_(0),
    joint_names_({ "MFRAME", "BLOCK", "BOOM", "ARM", "TOHKU_PITCH", "TOHKU_ROLL", 
                    "UFRAME", "MNP_SWING", "MANIBOOM", "MANIARM","MANIELBOW", 
                    "YAWJOINT", "HANDBASE", "TOHKU_TIP_01", "TOHKU_TIP_02", "PUSHROD" }),
    tohoku_gipper_joint_names_({"TOHKU_TIP_01", "TOHKU_TIP_02"}),
    mani_gipper_joint_names_({"PUSHROD"})
{
  joy_cmd_sub_ = nh_.subscribe("joy", 10, &RTRJoycon::updateJoyMsg, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RTRDoubleArmV7/base_controller/cmd_vel", 10);
  jog_joint_pub_ = nh_.advertise<jog_msgs::JogJoint>("/jog_joint", 10);
  tohoku_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/RTRDoubleArmV7/tohoku_gripper_controller/command", 10);
  mani_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/RTRDoubleArmV7/mani_gripper_controller/command", 10);

  joy_msg_.axes = {0};
  joy_msg_.buttons = {0};
}

void RTRJoycon::readYaml(const std::string config)
{
  std::string path = "/" + config + "/";
  if (nh_.getParam(path + "stick_left_H",       key_config_.at(0)));
  if (nh_.getParam(path + "stick_left_V",       key_config_.at(1)));
  if (nh_.getParam(path + "button_l2",          key_config_.at(2)));
  if (nh_.getParam(path + "stick_right_H",      key_config_.at(3)));
  if (nh_.getParam(path + "stick_right_V",      key_config_.at(4)));
  if (nh_.getParam(path + "button_r2",          key_config_.at(5)));
  if (nh_.getParam(path + "button_left_right",  key_config_.at(6)));
  if (nh_.getParam(path + "button_up_down",     key_config_.at(7)));
  if (nh_.getParam(path + "button_A",           key_config_.at(8)));
  if (nh_.getParam(path + "button_B",           key_config_.at(9)));
  if (nh_.getParam(path + "button_Y",           key_config_.at(10)));
  if (nh_.getParam(path + "button_X",           key_config_.at(11)));
  if (nh_.getParam(path + "button_l1",          key_config_.at(12)));
  if (nh_.getParam(path + "button_r1",          key_config_.at(13)));
  if (nh_.getParam(path + "button_l2",          key_config_.at(14)));
  if (nh_.getParam(path + "button_r2",          key_config_.at(15)));
  if (nh_.getParam(path + "button_option",      key_config_.at(16)));
  if (nh_.getParam(path + "button_start",       key_config_.at(17)));
  if (nh_.getParam(path + "button_home",        key_config_.at(18)));
  if (nh_.getParam(path + "button_l3",          key_config_.at(19)));
  if (nh_.getParam(path + "button_r3",          key_config_.at(20)));
}

void RTRJoycon::updateJoyMsg(const sensor_msgs::Joy& joy_msg)
{
  joy_msg_ = joy_msg;
}
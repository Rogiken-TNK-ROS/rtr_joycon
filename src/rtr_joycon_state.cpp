#include <rtr_joycon/rtr_joycon_state.h>

RtrJoyconState::RtrJoyconState(ros::NodeHandle& nh)
  : nh_(nh),
    speed_rate_(0.2),
    joy_config_(config_0()), 
    config_num_(0),
    axes_(9, ""),
    buttons_(14, ""),
    joint_names_({ "MFRAME", "BLOCK", "BOOM", "ARM", "TOHKU_PITCH", "TOHKU_ROLL", 
                    "UFRAME", "MNP_SWING", "MANIBOOM", "MANIARM","MANIELBOW", 
                    "YAWJOINT", "HANDBASE", "TOHKU_TIP_01", "TOHKU_TIP_02", "PUSHROD" })
{
  joy_node_sub_ = nh_.subscribe("joy", 10, &RtrJoyconState::updateJoyMsg, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RTRDoubleArmV7/base_controller/cmd_vel", 10);
  jog_joint_pub_ = nh_.advertise<jog_msgs::JogJoint>("/jog_joint", 10);

  joy_msg_.axes = {0};
  joy_msg_.buttons = {0};
  joy_msg_buf_.axes = {0};
  joy_msg_buf_.buttons = {0};
}

void RtrJoyconState::updateJoyMsg(const sensor_msgs::Joy& joy_msg)
{
  joy_msg_buf_ = joy_msg_;
  joy_msg_ = joy_msg;

  updateState();
}

void RtrJoyconState::publish(void)
{
  // cmd_vel
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x  = joy_msg_.axes[getStringIndex("STEERING_X")];
  cmd_vel.angular.z = joy_msg_.axes[getStringIndex("STEERING_Z")];
  cmd_vel_pub_.publish(cmd_vel);

  // jog_joint
  jog_msgs::JogJoint jog_joint;
  for (auto joint_name : joint_names_)
  {
    jog_joint.joint_names.push_back(joint_name);
    jog_joint.deltas.push_back(speed_rate_ * joy_msg_.axes[getStringIndex(joint_name)]);
  }
  jog_joint_pub_.publish(jog_joint);
}

int RtrJoyconState::getStringIndex(const std::string string)
{
  int index = 0;
  while (axes_.size() > index)
  {
    if (axes_[index] == string) break;
    index++;
  }
  return index;
}

void RtrJoyconState::readYaml(const std::string config)
{
  std::string path = "/" + config + "/";
  if (nh_.getParam(path + "stick_left_H",       axes_.at(0)));
  if (nh_.getParam(path + "stick_left_V",       axes_.at(1)));
  if (nh_.getParam(path + "button_l2",          axes_.at(2)));
  if (nh_.getParam(path + "stick_right_H",      axes_.at(3)));
  if (nh_.getParam(path + "stick_right_V",      axes_.at(4)));
  if (nh_.getParam(path + "button_r2",          axes_.at(5)));
  if (nh_.getParam(path + "button_left_right",  axes_.at(6)));
  if (nh_.getParam(path + "button_up_down",     axes_.at(7)));
  if (nh_.getParam(path + "button_A",           buttons_.at(0)));
  if (nh_.getParam(path + "button_B",           buttons_.at(1)));
  if (nh_.getParam(path + "button_Y",           buttons_.at(2)));
  if (nh_.getParam(path + "button_X",           buttons_.at(3)));
  if (nh_.getParam(path + "button_l1",          buttons_.at(4)));
  if (nh_.getParam(path + "button_r1",          buttons_.at(5)));
  if (nh_.getParam(path + "button_l2",          buttons_.at(6)));
  if (nh_.getParam(path + "button_r2",          buttons_.at(7)));
  if (nh_.getParam(path + "button_option",      buttons_.at(8)));
  if (nh_.getParam(path + "button_start",       buttons_.at(9)));
  if (nh_.getParam(path + "button_home",        buttons_.at(10)));
  if (nh_.getParam(path + "button_l3",          buttons_.at(11)));
  if (nh_.getParam(path + "button_r3",          buttons_.at(12)));
}

void RtrJoyconState::updateState(void)
{
  if (joy_msg_buf_.buttons[8] < joy_msg_.buttons[8])
  {
    config_num_++;
    if (config_num_ > 2) config_num_ = 0;
  }

  // switch pattern
  switch (config_num_)
  {
    case 0:
      joy_config_ = config_0();
      readYaml("config_0");
			break;
    case 1:
      joy_config_ = config_1();
      readYaml("config_1");
			break;
    case 2:
      joy_config_ = config_2();
      readYaml("config_2");
			break;
    default:
			break;
  }
}

void RtrJoyconState::WriteString(std::string const& s)
{
  joy_config_(s);
}

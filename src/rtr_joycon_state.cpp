#include <rtr_joycon/rtr_joycon_state.h>

RtrJoyconState::RtrJoyconState(ros::NodeHandle& nh)
  : nh_(nh), 
    joy_config_(config_0()), 
    config_num_(0),
    axes_(8, ""),
    buttons_(13, "")
{
  joy_node_sub_ = nh_.subscribe("joy", 10, &RtrJoyconState::updateJoyMsg, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RTRDoubleArmV7/base_controller/cmd_vel", 10);

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
  int i = 0;
  while (axes_.at(i) != "/RTRDoubleArmV7/base_controller/cmd_vel")
  {
    i++;
    if (i>7) break;
  }
  std::cout << "index:" << i << std::endl;
  cmd_vel_.linear.x = joy_msg_.axes[i];
  cmd_vel_pub_.publish(cmd_vel_);
}

void RtrJoyconState::setKeyAssign(const std::string config)
{
  std::string path = "/" + config + "/";
  if (nh_.getParam(path + "stick_left", axes_.at(0)));
  if (nh_.getParam(path + "stick_left", axes_.at(1)));
  if (nh_.getParam(path + "button_l2", axes_.at(2)));
  if (nh_.getParam(path + "stick_right", axes_.at(3)));
  if (nh_.getParam(path + "stick_right", axes_.at(4)));
  if (nh_.getParam(path + "button_r2", axes_.at(5)));
  if (nh_.getParam(path + "button_left_right", axes_.at(6)));
  if (nh_.getParam(path + "button_up_down", axes_.at(7)));
  if (nh_.getParam(path + "button_A", buttons_.at(0)));
  if (nh_.getParam(path + "button_B", buttons_.at(1)));
  if (nh_.getParam(path + "button_Y", buttons_.at(2)));
  if (nh_.getParam(path + "button_X", buttons_.at(3)));
  if (nh_.getParam(path + "button_l1", buttons_.at(4)));
  if (nh_.getParam(path + "button_r1", buttons_.at(5)));
  if (nh_.getParam(path + "button_l2", buttons_.at(6)));
  if (nh_.getParam(path + "button_r2", buttons_.at(7)));
  if (nh_.getParam(path + "button_option", buttons_.at(8)));
  if (nh_.getParam(path + "button_start", buttons_.at(9)));
  if (nh_.getParam(path + "button_home", buttons_.at(10)));
  if (nh_.getParam(path + "button_l3", buttons_.at(11)));
  if (nh_.getParam(path + "button_r3", buttons_.at(12)));
}

void RtrJoyconState::updateState(void)
{
  if (joy_msg_buf_.buttons[8] < joy_msg_.buttons[8])
  {
    config_num_++;
    if (config_num_ > 1) config_num_ = 0;
  }

  // switch pattern
  switch (config_num_)
  {
    case 0:
      joy_config_ = config_0();
      // setKeyAssign("config_0");
			break;
    case 1:
      joy_config_ = config_1();
      // setKeyAssign("config_1");
			break;
    default:
      joy_config_ = joy_config_;
			break;
  }
}

void RtrJoyconState::WriteString(std::string const& s)
{
  joy_config_(s);
}

#include <rtr_joycon/rtr_joycon_state.h>

RtrJoyconState::RtrJoyconState(ros::NodeHandle& nh)
 : nh_(nh), joy_config_(config_1()), count_(0)
{
  joy_node_sub_ = nh.subscribe("joy", 10, &RtrJoyconState::updateState, this);
}

void RtrJoyconState::updateState(const sensor_msgs::Joy& joy_msg)
{
  if (count_ < joy_msg.buttons[6])
  {
    joy_config_ = config_2();
  }
  else if (count_ > joy_msg.buttons[6])
  {
    joy_config_ = config_1();
  }
  else
  {
    joy_config_ = joy_config_;
  }
  
  
  count_ = joy_msg.buttons[6];
}

void RtrJoyconState::WriteString(std::string const& s)
{
  joy_config_(s);
}

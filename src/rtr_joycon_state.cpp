#include <rtr_joycon/rtr_joycon_state.h>

RtrJoyconState::RtrJoyconState(ros::NodeHandle& nh)
 : nh_(nh), joy_config_(config_1()), count_(0), state_num_(0)
{
  joy_node_sub_ = nh.subscribe("joy", 10, &RtrJoyconState::updateState, this);
}

void RtrJoyconState::updateState(const sensor_msgs::Joy& joy_msg)
{
  if (count_ < joy_msg.buttons[6])
  {
    state_num_ += 1;
    if (state_num_ > 1)
    { 
      state_num_ = 0;
    }
  }

  std::cout << state_num_ << std::endl;

  // // switch pattern
  switch (state_num_)
  {
    case 0:
      joy_config_ = config_1();
			break;
    case 1:
      joy_config_ = config_2();
			break;
    default:
      joy_config_ = joy_config_;
			break;
  }

  // // if pattern
  // if (state_num_ == 0)
  // {
  //   joy_config_ = config_1();
  // }
  // else if (state_num_ == 1)
  // {
  //   joy_config_ = config_2();
  // }
  
  count_ = joy_msg.buttons[6];
}

void RtrJoyconState::WriteString(std::string const& s)
{
  joy_config_(s);
}

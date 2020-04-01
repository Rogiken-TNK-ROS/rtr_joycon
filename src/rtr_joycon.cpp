#include <rtr_joycon/rtr_joycon.h>

RTRJoycon::RTRJoycon(ros::NodeHandle& nh)
  : nh_(nh),
    key_config_index_(0),
    key_config_(22, ""),
    double_arm_joint_names_(readYaml("RTRDoubleArmV7/rtr_double_arm/joints/")),
    tohoku_gipper_joint_names_(readYaml("RTRDoubleArmV7/tohoku_gipper/joints/")),
    mani_gipper_joint_names_(readYaml("RTRDoubleArmV7/mani_gipper/joints/"))
{
  joy_cmd_sub_ = nh_.subscribe("joy", 10, &RTRJoycon::updateJoyMsg, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RTRDoubleArmV7/base_controller/cmd_vel", 10);
  jog_frame_pub_ = nh_.advertise<jog_msgs::JogFrame>("/jog_frame", 10);

  joy_msg_.axes = {0};
  joy_msg_.buttons = {0};
  if (!getKeyConfig(key_config_index_)) ROS_ERROR("Failed to get key_config.");
}

void RTRJoycon::updateJoyMsg(const sensor_msgs::Joy& joy_msg)
{
  // optionボタンが押下されたときの処理
  int option_index = 9;
  if (joy_msg_.buttons[option_index] < joy_msg.buttons[option_index])
  {
    key_config_index_++;
    if (2 < key_config_index_) key_config_index_ = 0;   // reset config index
    if (!getKeyConfig(key_config_index_)) ROS_ERROR("Failed to get key_config.");
  }
  
  // update member variable
  joy_msg_ = joy_msg;
}

void RTRJoycon::publish(void)
{
  // std::cout << double_arm_joint_names_.size() << std::endl;
  // std::cout << tohoku_gipper_joint_names_.size() << std::endl;
  // std::cout << mani_gipper_joint_names_.size() << std::endl;
  // std::cout << key_config_.size() << std::endl;
  // std::cout << key_config_index_ << std::endl;
  // for (int i=0;i<22;i++)
  //   std::cout << key_config_[i] << std::endl;
}

// pathに該当するyaml内の値を返す関数
std::vector<std::string> RTRJoycon::readYaml(const std::string path)
{
  std::vector<std::string> param_list;
  if (!nh_.getParam(path, param_list))
    ROS_ERROR("Failed to get parameter from yaml file.");
  return param_list;
}

// キーコンフィグを取得する関数
bool RTRJoycon::getKeyConfig(const int config_num)
{
  std::string num;
  num = std::to_string(config_num);
  std::cout << "key_config setting is -> " << config_num << std::endl;
  std::string config_path = "/config_" + std::to_string(config_num) + "/";
  if (!nh_.getParam(config_path + "stick_left_H",   key_config_.at(0))) return false;
  if (!nh_.getParam(config_path + "stick_left_V",   key_config_.at(1))) return false;
  if (!nh_.getParam(config_path + "button_l2",      key_config_.at(2))) return false;
  if (!nh_.getParam(config_path + "stick_right_H",  key_config_.at(3))) return false;
  if (!nh_.getParam(config_path + "stick_right_V",  key_config_.at(4))) return false;
  if (!nh_.getParam(config_path + "button_r2",      key_config_.at(5))) return false;
  if (!nh_.getParam(config_path + "button_H",       key_config_.at(6))) return false;
  if (!nh_.getParam(config_path + "button_V",       key_config_.at(7))) return false;
  if (!nh_.getParam(config_path + "button_A",       key_config_.at(8))) return false;
  if (!nh_.getParam(config_path + "button_B",       key_config_.at(9))) return false;
  if (!nh_.getParam(config_path + "button_Y",       key_config_.at(10))) return false;
  if (!nh_.getParam(config_path + "button_X",       key_config_.at(11))) return false;
  if (!nh_.getParam(config_path + "button_l1",      key_config_.at(12))) return false;
  if (!nh_.getParam(config_path + "button_r1",      key_config_.at(13))) return false;
  if (!nh_.getParam(config_path + "button_l2",      key_config_.at(14))) return false;
  if (!nh_.getParam(config_path + "button_r2",      key_config_.at(15))) return false;
  if (!nh_.getParam(config_path + "button_option",  key_config_.at(16))) return false;
  if (!nh_.getParam(config_path + "button_start",   key_config_.at(17))) return false;
  if (!nh_.getParam(config_path + "button_home",    key_config_.at(18))) return false;
  if (!nh_.getParam(config_path + "button_l3",      key_config_.at(19))) return false;
  if (!nh_.getParam(config_path + "button_r3",      key_config_.at(20))) return false;

  return true;
}

bool RTRJoycon::getStringIndex(int &index, 
                               const std::vector<std::string> str_vec, 
                               const std::string string)
{
  index = 0;
  while (str_vec.size() > index)
  {
    if (str_vec[index] == string) break;
    index++;
  }

  if (index == str_vec.size()) return false;
  else return true;
}

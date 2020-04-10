#include <rtr_joycon/rtr_joycon.h>

RTRJoycon::RTRJoycon(ros::NodeHandle& nh)
  : nh_(nh),
    speed_gain_(readYaml<float>("/setting_0/speed_gain")),
    double_arm_joint_names_(readYaml<std::vector<std::string>>("/RTRDoubleArmV7/rtr_double_arm/joints/")),
    tohoku_gipper_joint_names_(readYaml<std::vector<std::string>>("/RTRDoubleArmV7/tohoku_gipper/joints/")),
    mani_gipper_joint_names_(readYaml<std::vector<std::string>>("/RTRDoubleArmV7/mani_gipper/joints/"))
{
  joy_cmd_sub_ = nh_.subscribe("/joy", 10, &RTRJoycon::updateJoyMsg, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RTRDoubleArmV7/base_controller/cmd_vel", 10);
  jog_joint_pub_ = nh_.advertise<jog_msgs::JogJoint>("/jog_joint", 10);
  jog_frame_pub_ = nh_.advertise<jog_msgs::JogFrame>("/jog_frame", 10);
  L1_enable_pub_ = nh_.advertise<std_msgs::Bool>("/rtr_joycon/L1_enable", 10);
  R1_enable_pub_ = nh_.advertise<std_msgs::Bool>("/rtr_joycon/R1_enable", 10);
  mani_arm_enable_pub_ = nh_.advertise<std_msgs::Bool>("/rtr_joycon/mani_arm_enable", 10);

  for (auto joint_name : double_arm_joint_names_)
  {
    jog_joint_.joint_names.push_back(joint_name);
    jog_joint_.deltas.push_back(0);
  }

  L1_enable_.data = false;
  R1_enable_.data = false;
  mani_arm_enable_.data = false;

  joy_msg_.axes = {0.0};
  joy_msg_.buttons = {0};
}

void RTRJoycon::updateJoyMsg(const sensor_msgs::Joy& joy_msg)
{
  int index;

  // optionボタンが押下された場合の処理
  index = readYaml<int>("/keymap/buttons/option_button");
  if (joy_msg_.buttons[index] < joy_msg.buttons[index])
  {
    if (!mani_arm_enable_.data) mani_arm_enable_.data = true;
    else mani_arm_enable_.data = false;
  }

  // L1ボタンが押下された場合の処理
  index = readYaml<int>("/keymap/buttons/L1_button");
  if (joy_msg.buttons[index]) L1_enable_.data = true;
  else L1_enable_.data = false;

  // R1ボタンが押下された場合の処理
  index = readYaml<int>("/keymap/buttons/R1_button");
  if (joy_msg.buttons[index]) R1_enable_.data = true;
  else R1_enable_.data = false;
  
  // update member variable
  joy_msg_ = joy_msg;
}

void RTRJoycon::publish(void)
{
  L1_enable_pub_.publish(L1_enable_);
  R1_enable_pub_.publish(R1_enable_);
  mani_arm_enable_pub_.publish(mani_arm_enable_);
  cmd_vel_pub_.publish(cmd_vel_);
  jog_joint_pub_.publish(jog_joint_);
}

void RTRJoycon::updateCommand(void)
{
  if (L1_enable_.data)
  {
    if (!setCommand("L1andL_stick_V",  joy_msg_.axes[readYaml<int>("/keymap/axes/L_stick_V")]*speed_gain_, 0.0)) ROS_ERROR("L1andL_stick_V task was FAILED");    // crawler_move
    if (!setCommand("L1andR_stick_H",  joy_msg_.axes[readYaml<int>("/keymap/axes/R_stick_H")]*speed_gain_, 0.0)) ROS_ERROR("L1andR_stick_H task was FAILED");    // crawler_turn
    if (!setCommand("L1andX_B_button", joy_msg_.buttons[readYaml<int>("/keymap/buttons/X_button")]*speed_gain_, joy_msg_.buttons[readYaml<int>("/keymap/buttons/B_button")])*speed_gain_) ROS_ERROR("L1andX_B_button task was FAILED");   // end_effector_ROLL
  }
  else
  {
    if (!setCommand("cross_key_V", joy_msg_.axes[readYaml<int>("/keymap/axes/cross_key_V")]*speed_gain_, 0.0)) ROS_ERROR("cross_key_V task was FAILED");     // crawler_move
    if (!setCommand("cross_key_H", joy_msg_.axes[readYaml<int>("/keymap/axes/cross_key_H")]*speed_gain_, 0.0)) ROS_ERROR("cross_key_H task was FAILED");     // crawler_turn
    if (!setCommand("L_stick_V", joy_msg_.axes[readYaml<int>("/keymap/axes/L_stick_V")]*speed_gain_, 0.0)) ROS_ERROR("L_stick_V task was FAILED");       // 1st_joint_PITCH
    if (!setCommand("R_stick_H", joy_msg_.axes[readYaml<int>("/keymap/axes/R_stick_H")]*speed_gain_, 0.0)) ROS_ERROR("R_stick_H task was FAILED");       // 1st_joint_YAW
    if (!setCommand("X_B_button", joy_msg_.buttons[readYaml<int>("/keymap/buttons/X_button")]*speed_gain_, joy_msg_.buttons[readYaml<int>("/keymap/buttons/B_button")])*speed_gain_) ROS_ERROR("X_B_button task was FAILED");      // end_effector_YAW
  }
  
  if (R1_enable_.data)
  {
    if (!setCommand("R1andR2", joy_msg_.axes[readYaml<int>("/keymap/axes/r2_triger")]*speed_gain_, 0.0)) ROS_ERROR("R1andR2 task was FAILED");           // end_effector_motion
  }
  
  if (!setCommand("L_stick_H", joy_msg_.axes[readYaml<int>("/keymap/axes/L_stick_H")]*speed_gain_, 0.0)) ROS_ERROR("stick_H task was FAILED");         // arm_base_YAW
  if (!setCommand("R_stick_V", joy_msg_.axes[readYaml<int>("/keymap/axes/R_stick_V")]*speed_gain_, 0.0)) ROS_ERROR("stick_V task was FAILED");         // 2nd_joint_PITCH
  if (!setCommand("Y_A_button", joy_msg_.buttons[readYaml<int>("/keymap/buttons/Y_button")]*speed_gain_, joy_msg_.buttons[readYaml<int>("/keymap/buttons/A_button")])*speed_gain_) ROS_ERROR("A_button task was FAILED");        // end_effector_PITCH
}

bool RTRJoycon::setCommand(std::string task, float positive=0.0, float negative=0.0)
{
  const std::string path = "/setting_0/" + task;
  const std::string fuga = readYaml<std::string>(path);

  if (fuga == "crawler_move")
  {
    cmd_vel_.linear.x  = positive - abs(negative);
    return true;
  }
  else if (fuga == "crawler_turn")
  {
    cmd_vel_.angular.z = positive - abs(negative);
    return true;
  }
  else if (fuga == "arm_base_YAW")
  {
    std::string str;
    if (!mani_arm_enable_.data) str = "MFRAME";
    else str = "UFRAME";
    jog_joint_.deltas[getIndexFromVector(double_arm_joint_names_, str)] = positive - abs(negative);
    return true;
  }
  else if (fuga == "1st_joint_YAW")
  {
    std::string str;
    if (!mani_arm_enable_.data) str = "BLOCK";
    else str = "MNP_SWING";
    jog_joint_.deltas[getIndexFromVector(double_arm_joint_names_, str)] = positive - abs(negative);
    return true;
  }
  else if (fuga == "1st_joint_PITCH")
  {
    std::string str;
    if (!mani_arm_enable_.data) str = "BOOM";
    else str = "MANIBOOM";
    jog_joint_.deltas[getIndexFromVector(double_arm_joint_names_, str)] = positive - abs(negative);
    return true;
  }
  else if (fuga == "2nd_joint_PITCH")
  {
    std::string str;
    if (!mani_arm_enable_.data) str = "ARM";
    else str = "MANIARM";
    jog_joint_.deltas[getIndexFromVector(double_arm_joint_names_, str)] = positive - abs(negative);
    return true;
  }
  else if (fuga == "end_effector_PITCH")
  {
    std::string str;
    if (!mani_arm_enable_.data) str = "TOHKU_PITCH";
    else str = "MANIELBOW";
    jog_joint_.deltas[getIndexFromVector(double_arm_joint_names_, str)] = positive - abs(negative);
    return true;
  }
  else if (fuga == "end_effector_YAW")
  {
    std::string str;
    if (!mani_arm_enable_.data) str = "TOHKU_ROLL";
    else str = "YAWJOINT";
    jog_joint_.deltas[getIndexFromVector(double_arm_joint_names_, str)] = positive - abs(negative);
    return true;
  }
  else if (fuga == "end_effector_ROLL")
  {
    std::string str;
    if (!mani_arm_enable_.data) str = "TOHKU_ROLL";
    else str = "HANDBASE";
    jog_joint_.deltas[getIndexFromVector(double_arm_joint_names_, str)] = positive - abs(negative);
    return true;
  }
  else if (fuga == "end_effector_motion")
  {
    if (!mani_arm_enable_.data)
    {
      // joint trajectory for tohoku_gipper
      trajectory_.points.clear();
      trajectory_.joint_names.clear();
      trajectory_point_.positions.clear();
      for (auto joint_name : tohoku_gipper_joint_names_)
      {
        trajectory_.joint_names.push_back(joint_name);
        trajectory_point_.positions.push_back(0.349065850399 / 2.0 * (joy_msg_.axes[5] - 1.0));
      }
      ros::Duration time(1.0);
      trajectory_point_.time_from_start = time;
      trajectory_.points.push_back(trajectory_point_);

      return true;
    }
    else
    {
      // joint trajectory for mani_gipper
      trajectory_.points.clear();
      trajectory_.joint_names.clear();
      trajectory_point_.positions.clear();
      for (auto joint_name : mani_gipper_joint_names_)
      {
        trajectory_.joint_names.push_back(joint_name);
        trajectory_point_.positions.push_back(0.05 / 2.0 * (joy_msg_.axes[5] - 1.0));
      }
      ros::Duration time(1.0);
      trajectory_point_.time_from_start = time;
      trajectory_.points.push_back(trajectory_point_);
    }
  }

  return false;
}
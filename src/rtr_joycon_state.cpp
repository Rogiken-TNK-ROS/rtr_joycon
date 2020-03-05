#include <rtr_joycon/rtr_joycon_state.h>

RtrJoyconState::RtrJoyconState(ros::NodeHandle& nh)
  : nh_(nh),
    speed_rate_(0.2),
    joy_config_(config_0()), 
    config_num_(0),
    settings_(22, ""),
    joint_names_({ "MFRAME", "BLOCK", "BOOM", "ARM", "TOHKU_PITCH", "TOHKU_ROLL", 
                    "UFRAME", "MNP_SWING", "MANIBOOM", "MANIARM","MANIELBOW", 
                    "YAWJOINT", "HANDBASE", "TOHKU_TIP_01", "TOHKU_TIP_02", "PUSHROD" }),
    tohoku_gipper_joint_names_({"TOHKU_TIP_01", "TOHKU_TIP_02"}),
    mani_gipper_joint_names_({"PUSHROD"})
{
  joy_node_sub_ = nh_.subscribe("joy", 10, &RtrJoyconState::updateJoyMsg, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RTRDoubleArmV7/base_controller/cmd_vel", 10);
  jog_joint_pub_ = nh_.advertise<jog_msgs::JogJoint>("/jog_joint", 10);
  tohoku_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/RTRDoubleArmV7/tohoku_gripper_controller/command", 10);
  mani_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/RTRDoubleArmV7/mani_gripper_controller/command", 10);

  joy_msg_.axes = {0};
  joy_msg_.buttons = {0};
  joy_msg_buf_.axes = {0};
  joy_msg_buf_.buttons = {0};
}

void RtrJoyconState::publish(void)
{
  // cmd_vel
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x  = joy_msg_.axes[getStringIndex(settings_, "STEERING_X")];
  cmd_vel.angular.z = joy_msg_.axes[getStringIndex(settings_, "STEERING_Z")];
  cmd_vel_pub_.publish(cmd_vel);

  // jog_msgs
  jog_msgs::JogJoint jog_joint;
  for (auto joint_name : joint_names_)
  {
    jog_joint.joint_names.push_back(joint_name);
    jog_joint.deltas.push_back(speed_rate_ * jogCommandSet(joint_name));
  }
  jog_joint_pub_.publish(jog_joint);

  // tohoku_gripper
  trajectory_msgs::JointTrajectory tohoku_trajectory;
  trajectory_msgs::JointTrajectoryPoint tohoku_point;
  // mani_gripper
  trajectory_msgs::JointTrajectory mani_trajectory;
  trajectory_msgs::JointTrajectoryPoint mani_point;

  if (config_num_ == 1)
  {
    for (auto joint_name : tohoku_gipper_joint_names_)
    {
      tohoku_trajectory.joint_names.push_back(joint_name);
      tohoku_point.positions.push_back(0.349065850399 / 2.0 * (jogCommandSet("TOHKU_TIP_01") - 1.0));
    }
    ros::Duration time(1.0);
    tohoku_point.time_from_start = time;
    tohoku_trajectory.points.push_back(tohoku_point);
  }
  else if (config_num_ == 2)
  {
    for (auto joint_name : mani_gipper_joint_names_)
    {
      mani_trajectory.joint_names.push_back(joint_name);
      mani_point.positions.push_back(0.05 / 2.0 * (jogCommandSet("PUSHROD") - 1.0));
    }
    ros::Duration time(1.0);
    mani_point.time_from_start = time;
    mani_trajectory.points.push_back(mani_point);
  }
  else
  {
    /* code */
  }

  tohoku_trajectory_pub_.publish(tohoku_trajectory);
  mani_trajectory_pub_.publish(mani_trajectory);
}

float RtrJoyconState::jogCommandSet(const std::string joint_name)
{
  int axes_size = joy_msg_.axes.size();

  if (joint_name == "TOHKU_PITCH")
  {
    int TOHKU_PITCH_index = getStringIndex(settings_, "TOHKU_PITCH_UP");

    if (TOHKU_PITCH_index < axes_size)
      return joy_msg_.axes[getStringIndex(settings_, "TOHKU_PITCH_UP")] - joy_msg_.axes[getStringIndex(settings_, "TOHKU_PITCH_DOWN")];
    else
      return joy_msg_.buttons[getStringIndex(settings_, "TOHKU_PITCH_UP")-axes_size] - joy_msg_.buttons[getStringIndex(settings_, "TOHKU_PITCH_DOWN")-axes_size];
  }
  else if (joint_name == "TOHKU_ROLL")
  {
    int TOHKU_ROLL_index = getStringIndex(settings_, "TOHKU_ROLL_UP");

    if (TOHKU_ROLL_index < axes_size)
      return joy_msg_.axes[getStringIndex(settings_, "TOHKU_ROLL_UP")] - joy_msg_.axes[getStringIndex(settings_, "TOHKU_ROLL_DOWN")];
    else
      return joy_msg_.buttons[getStringIndex(settings_, "TOHKU_ROLL_UP")-axes_size] - joy_msg_.buttons[getStringIndex(settings_, "TOHKU_ROLL_DOWN")-axes_size];
  }
  else if (joint_name == "MANIELBOW")
  {
    int MANIELBOW_index = getStringIndex(settings_, "MANIELBOW_UP");

    if (MANIELBOW_index < axes_size)
      return joy_msg_.axes[getStringIndex(settings_, "MANIELBOW_UP")] - joy_msg_.axes[getStringIndex(settings_, "MANIELBOW_DOWN")];
    else
      return joy_msg_.buttons[getStringIndex(settings_, "MANIELBOW_UP")-axes_size] - joy_msg_.buttons[getStringIndex(settings_, "MANIELBOW_DOWN")-axes_size];
  }
  else if (joint_name == "YAWJOINT")
  {
    int YAWJOINT_index = getStringIndex(settings_, "YAWJOINT_UP");

    if (YAWJOINT_index < axes_size)
      return joy_msg_.axes[getStringIndex(settings_, "YAWJOINT_UP")] - joy_msg_.axes[getStringIndex(settings_, "YAWJOINT_DOWN")];
    else
      return joy_msg_.buttons[getStringIndex(settings_, "YAWJOINT_UP")-axes_size] - joy_msg_.buttons[getStringIndex(settings_, "YAWJOINT_DOWN")-axes_size];
  }
  else if (joint_name == "TOHKU_TIP_01" || joint_name == "TOHKU_TIP_02")
  {
    int TOHKU_TIP_index = getStringIndex(settings_, "TOHKU_TIP");

    if (TOHKU_TIP_index < axes_size)
      return joy_msg_.axes[TOHKU_TIP_index];
    else
      return joy_msg_.buttons[TOHKU_TIP_index - axes_size];
  }
  else if (joint_name == "PUSHROD")
  {
    int PUSHROD_index = getStringIndex(settings_, "PUSHROD");

    if (PUSHROD_index < axes_size)
      return joy_msg_.axes[PUSHROD_index];
    else
      return joy_msg_.buttons[PUSHROD_index-axes_size];
  }
  else
  {
    int index = getStringIndex(settings_, joint_name);

    if (index == 22)
      return 0.0;
    else if (index < axes_size)
      return joy_msg_.axes[getStringIndex(settings_, joint_name)];
    else
      return joy_msg_.buttons[getStringIndex(settings_, joint_name)-axes_size];
  }
  
}

int RtrJoyconState::getStringIndex(const std::vector<std::string> str_vec, const std::string string)
{
  int index = 0;
  while (str_vec.size() > index)
  {
    if (str_vec[index] == string) break;
    index++;
  }

  return index;
}

void RtrJoyconState::readYaml(const std::string config)
{
  std::string path = "/" + config + "/";
  if (nh_.getParam(path + "stick_left_H",       settings_.at(0)));
  if (nh_.getParam(path + "stick_left_V",       settings_.at(1)));
  if (nh_.getParam(path + "button_l2",          settings_.at(2)));
  if (nh_.getParam(path + "stick_right_H",      settings_.at(3)));
  if (nh_.getParam(path + "stick_right_V",      settings_.at(4)));
  if (nh_.getParam(path + "button_r2",          settings_.at(5)));
  if (nh_.getParam(path + "button_left_right",  settings_.at(6)));
  if (nh_.getParam(path + "button_up_down",     settings_.at(7)));
  if (nh_.getParam(path + "button_A",           settings_.at(8)));
  if (nh_.getParam(path + "button_B",           settings_.at(9)));
  if (nh_.getParam(path + "button_Y",           settings_.at(10)));
  if (nh_.getParam(path + "button_X",           settings_.at(11)));
  if (nh_.getParam(path + "button_l1",          settings_.at(12)));
  if (nh_.getParam(path + "button_r1",          settings_.at(13)));
  if (nh_.getParam(path + "button_l2",          settings_.at(14)));
  if (nh_.getParam(path + "button_r2",          settings_.at(15)));
  if (nh_.getParam(path + "button_option",      settings_.at(16)));
  if (nh_.getParam(path + "button_start",       settings_.at(17)));
  if (nh_.getParam(path + "button_home",        settings_.at(18)));
  if (nh_.getParam(path + "button_l3",          settings_.at(19)));
  if (nh_.getParam(path + "button_r3",          settings_.at(20)));
}

void RtrJoyconState::updateJoyMsg(const sensor_msgs::Joy& joy_msg)
{
  joy_msg_buf_ = joy_msg_;
  joy_msg_ = joy_msg;
  updateState();
}

void RtrJoyconState::updateState(void)
{
  int max_config_num = 2;
  int STATE_index = getStringIndex(settings_, "STATE") - joy_msg_.axes.size();

  if (joy_msg_buf_.buttons[STATE_index] < joy_msg_.buttons[STATE_index])
  {
    config_num_++;
    if (config_num_ > max_config_num) config_num_ = 0;
  }

  // switch pattern
  switch (config_num_)
  {
    case 0:
      joy_config_ = config_0();
      ROS_INFO_STREAM("JoyCon config : 0");
      readYaml("config_0");
			break;
    case 1:
      joy_config_ = config_1();
      ROS_INFO_STREAM("JoyCon config : 1");
      readYaml("config_1");
			break;
    case 2:
      joy_config_ = config_2();
      ROS_INFO_STREAM("JoyCon config : 2");
      readYaml("config_2");
			break;
    default:
			break;
  }
}

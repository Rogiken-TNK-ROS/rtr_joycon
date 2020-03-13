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
    mani_gipper_joint_names_({"PUSHROD"}),
    group_names_({"tohoku_arm", "mani_arm"}),
    link_names_({"TOHKU_ROLL", "HANDBASE"})
{
  joy_node_sub_ = nh_.subscribe("joy", 10, &RtrJoyconState::updateJoyMsg, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RTRDoubleArmV7/base_controller/cmd_vel", 10);
  jog_joint_pub_ = nh_.advertise<jog_msgs::JogJoint>("/jog_joint", 10);
  jog_frame_pub_ = nh_.advertise<jog_msgs::JogFrame>("/jog_frame", 10);
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
  int steer_x_index;
  int steer_z_index;
  getStringIndex(steer_x_index, settings_, "STEERING_X");
  getStringIndex(steer_z_index, settings_, "STEERING_Z");
  if (steer_x_index == settings_.size())
  {
    cmd_vel.linear.x  = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  else 
  {
    cmd_vel.linear.x  = joy_msg_.axes[steer_x_index];
    cmd_vel.angular.z = joy_msg_.axes[steer_z_index];
  }
  cmd_vel_pub_.publish(cmd_vel);

  // // jog_msgs
  // jog_msgs::JogJoint jog_joint;
  // for (auto joint_name : joint_names_)
  // {
  //   jog_joint.joint_names.push_back(joint_name);
  //   jog_joint.deltas.push_back(speed_rate_ * jogCommandSet(joint_name));
  // }
  // jog_joint_pub_.publish(jog_joint);

  // jog_frame
  if (config_num_ == 1)
  {
    jog_msgs::JogFrame jog_frame;
    jog_frame.header.frame_id = "base_footprint";
    jog_frame.group_name = group_names_[0];
    jog_frame.link_name = link_names_[0];
    jog_frame.linear_delta.x = 0.01 * joy_msg_.axes[7];
    jog_frame.linear_delta.y = 0.01 * joy_msg_.axes[6];
    jog_frame.linear_delta.z = 0.01 * (joy_msg_.buttons[5] - joy_msg_.buttons[4]);
    jog_frame.angular_delta.x = 0.01 * (joy_msg_.buttons[1] - joy_msg_.buttons[3]);
    jog_frame.angular_delta.y = 0.01 * (joy_msg_.buttons[0] - joy_msg_.buttons[2]);
    jog_frame.angular_delta.z = 0.00;
    jog_frame.avoid_collisions = false;
    jog_frame.header.stamp = ros::Time::now();
    jog_frame_pub_.publish(jog_frame);
  }

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
    int TOHKU_PITCH_UP_index;
    int TOHKU_PITCH_DOWN_index;
    if (!getStringIndex(TOHKU_PITCH_UP_index, settings_,   "TOHKU_PITCH_UP") ||
        !getStringIndex(TOHKU_PITCH_DOWN_index, settings_, "TOHKU_PITCH_DOWN"))
    {
      return 0.0;
    }

    if (TOHKU_PITCH_UP_index < axes_size)
      return joy_msg_.axes[TOHKU_PITCH_UP_index] - joy_msg_.axes[TOHKU_PITCH_DOWN_index];
    else
      return joy_msg_.buttons[TOHKU_PITCH_UP_index-axes_size] - joy_msg_.buttons[TOHKU_PITCH_DOWN_index-axes_size];
  }
  else if (joint_name == "TOHKU_ROLL")
  {
    int TOHKU_ROLL_UP_index;
    int TOHKU_ROLL_DOWN_index;
    if (!getStringIndex(TOHKU_ROLL_UP_index, settings_,   "TOHKU_ROLL_UP") ||
        !getStringIndex(TOHKU_ROLL_DOWN_index, settings_, "TOHKU_ROLL_DOWN"))
    {
      return 0.0;
    }

    if (TOHKU_ROLL_UP_index < axes_size)
      return joy_msg_.axes[TOHKU_ROLL_UP_index] - joy_msg_.axes[TOHKU_ROLL_DOWN_index];
    else
      return joy_msg_.buttons[TOHKU_ROLL_UP_index-axes_size] - joy_msg_.buttons[TOHKU_ROLL_DOWN_index-axes_size];
  }
  else if (joint_name == "MANIELBOW")
  {
    int MANIELBOW_UP_index;
    int MANIELBOW_DOWN_index;
    if (!getStringIndex(MANIELBOW_UP_index, settings_,   "MANIELBOW_UP") ||
        !getStringIndex(MANIELBOW_DOWN_index, settings_, "MANIELBOW_DOWN"))
    {
      return 0.0;
    }

    if (MANIELBOW_UP_index < axes_size)
      return joy_msg_.axes[MANIELBOW_UP_index] - joy_msg_.axes[MANIELBOW_DOWN_index];
    else
      return joy_msg_.buttons[MANIELBOW_UP_index-axes_size] - joy_msg_.buttons[MANIELBOW_DOWN_index-axes_size];
  }
  else if (joint_name == "YAWJOINT")
  {
    int YAWJOINT_UP_index;
    int YAWJOINT_DOWN_index;
    if (!getStringIndex(YAWJOINT_UP_index, settings_,   "YAWJOINT_UP") ||
        !getStringIndex(YAWJOINT_DOWN_index, settings_, "YAWJOINT_DOWN"))
    {
      return 0.0;
    }

    if (YAWJOINT_UP_index < axes_size)
      return joy_msg_.axes[YAWJOINT_UP_index] - joy_msg_.axes[YAWJOINT_DOWN_index];
    else
      return joy_msg_.buttons[YAWJOINT_UP_index-axes_size] - joy_msg_.buttons[YAWJOINT_DOWN_index-axes_size];
  }
  else if (joint_name == "TOHKU_TIP_01" || joint_name == "TOHKU_TIP_02")
  {
    int TOHKU_TIP_index;
    if (!getStringIndex(TOHKU_TIP_index, settings_, "TOHKU_TIP"))
    {
      return 0.0;
    }

    if (TOHKU_TIP_index < axes_size)
      return joy_msg_.axes[TOHKU_TIP_index];
    else
      return joy_msg_.buttons[TOHKU_TIP_index - axes_size];
  }
  else if (joint_name == "PUSHROD")
  {
    int PUSHROD_index;
    if (!getStringIndex(PUSHROD_index, settings_, "PUSHROD"))
    {
      return 0.0;
    }

    if (PUSHROD_index < axes_size)
      return joy_msg_.axes[PUSHROD_index];
    else
      return joy_msg_.buttons[PUSHROD_index-axes_size];
  }
  else
  {
    int index;
    if (!getStringIndex(index, settings_, joint_name))
    {
      return 0.0;
    }

    if (index < axes_size)
      return joy_msg_.axes[index];
    else
      return joy_msg_.buttons[index-axes_size];
  }
  
}

bool RtrJoyconState::getStringIndex(int &index, 
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
  int index;
  getStringIndex(index, settings_, "STATE");
  int STATE_index = index - joy_msg_.axes.size();

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

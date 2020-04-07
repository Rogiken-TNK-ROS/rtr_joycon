#include <rtr_joycon/rtr_joycon.h>

RTRJoycon::RTRJoycon(ros::NodeHandle& nh)
  : nh_(nh),
    key_config_index_(0),
    joy_con_key_names_(readYaml("key_names/")),
    double_arm_joint_names_(readYaml("RTRDoubleArmV7/rtr_double_arm/joints/")),
    tohoku_gipper_joint_names_(readYaml("RTRDoubleArmV7/tohoku_gipper/joints/")),
    mani_gipper_joint_names_(readYaml("RTRDoubleArmV7/mani_gipper/joints/"))
{
  joy_cmd_sub_ = nh_.subscribe("joy", 10, &RTRJoycon::updateJoyMsg, this);
  jog_joint_pub_ = nh_.advertise<jog_msgs::JogJoint>("/jog_joint", 10);
  jog_frame_pub_ = nh_.advertise<jog_msgs::JogFrame>("/jog_frame", 10);

  joy_msg_.axes = {0};
  joy_msg_.buttons = {0};
  if (!setKeyConfig(key_config_index_)) ROS_ERROR("Failed to initialize key_config.");
}

void RTRJoycon::updateJoyMsg(const sensor_msgs::Joy& joy_msg)
{
  // optionボタンが押下された場合の処理
  const std::string OPTION = "OPTION";
  const int option_index = getStringIndex(key_config_, OPTION) - joy_msg_.axes.size() + 1;
  if (joy_msg_.buttons[option_index] < joy_msg.buttons[option_index])
  {
    // key_config_index_の切り替え & key_configの更新
    key_config_index_++;
    int index_max_num = 4;  // yamlに設定されているconfigの最大インデックス番号
    if (index_max_num < key_config_index_) key_config_index_ = 0;   // reset key_config index
    if (!setKeyConfig(key_config_index_)) ROS_ERROR("Failed to set key_config.");
  }
  
  // update member variable
  joy_msg_ = joy_msg;
}

void RTRJoycon::publish(void)
{
  std::string info_path;

  info_path = "/setting_" + std::to_string(key_config_index_) + "/setting_info/target";
  std::vector<std::string> target = readYaml(info_path);
  info_path = "/setting_" + std::to_string(key_config_index_) + "/setting_info/topic_type";
  std::vector<std::string> topic_type = readYaml(info_path);

  if (target[0] == "base_foot")
  {
    if (topic_type[0] == "cmd_vel")
    {
      // cmd_vel
      const std::string STEERING_X = "STEERING_X";
      const int steer_x_index = getStringIndex(key_config_, STEERING_X);
      const std::string STEERING_Z = "STEERING_Z";
      const int steer_z_index = getStringIndex(key_config_, STEERING_Z);
      if (steer_x_index < 0 || steer_z_index < 0)
      {
        cmd_vel_.linear.x  = 0.0;
        cmd_vel_.angular.z = 0.0;
      }
      else
      {
        cmd_vel_.linear.x  = joy_msg_.axes[steer_x_index];
        cmd_vel_.angular.z = joy_msg_.axes[steer_z_index];
      }

      // set publisher
      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RTRDoubleArmV7/base_controller/cmd_vel", 10);
      cmd_vel_pub_.publish(cmd_vel_);
    }
  }
  else if (target[0] == "tohoku_gipper")
  {    
    if (topic_type[0] == "jog_joint")
    {
      jog_joint_.joint_names.clear();
      jog_joint_.deltas.clear();
      for (auto joint_name : double_arm_joint_names_)
      {
        jog_joint_.joint_names.push_back(joint_name);
        jog_joint_.deltas.push_back(speed_gain_ * jogCommandSet(joint_name));
      }
      jog_joint_pub_.publish(jog_joint_);
    }
    else if (topic_type[0] == "jog_frame")
    {
      std::vector<std::string> group_names = readYaml("/RTRDoubleArmV7/tohoku_gipper/group_name/");
      std::vector<std::string> link_names = readYaml("/RTRDoubleArmV7/tohoku_gipper/link_name/");
      jog_frame_.header.frame_id = "base_footprint";
      jog_frame_.group_name = group_names[0];
      jog_frame_.link_name = link_names[0];
      jog_frame_.linear_delta.x = 0.01 * joy_msg_.axes[7];
      jog_frame_.linear_delta.y = 0.01 * joy_msg_.axes[6];
      jog_frame_.linear_delta.z = 0.01 * (joy_msg_.buttons[5] - joy_msg_.buttons[4]);
      jog_frame_.angular_delta.x = 0.01 * (joy_msg_.buttons[1] - joy_msg_.buttons[3]);
      jog_frame_.angular_delta.y = 0.01 * (joy_msg_.buttons[0] - joy_msg_.buttons[2]);
      jog_frame_.angular_delta.z = 0.00;
      jog_frame_.avoid_collisions = false;
      jog_frame_.header.stamp = ros::Time::now();
      jog_frame_pub_.publish(jog_frame_);
    }

    // joint trajectory for tohoku_gipper
    trajectory_.points.clear();
    trajectory_.joint_names.clear();
    trajectory_point_.positions.clear();
    for (auto joint_name : tohoku_gipper_joint_names_)
    {
      trajectory_.joint_names.push_back(joint_name);
      trajectory_point_.positions.push_back(0.349065850399 / 2.0 * (jogCommandSet("TOHKU_TIP_01") - 1.0));
    }
    ros::Duration time(1.0);
    trajectory_point_.time_from_start = time;
    trajectory_.points.push_back(trajectory_point_);

    // set publisher
    trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/RTRDoubleArmV7/tohoku_gripper_controller/command", 10);
    trajectory_pub_.publish(trajectory_);
  }
  else if (target[0] == "mani_gipper")
  {    
    if (topic_type[0] == "jog_joint")
    {
      jog_joint_.joint_names.clear();
      jog_joint_.deltas.clear();
      for (auto joint_name : double_arm_joint_names_)
      {
        jog_joint_.joint_names.push_back(joint_name);
        jog_joint_.deltas.push_back(speed_gain_ * jogCommandSet(joint_name));
      }
      jog_joint_pub_.publish(jog_joint_);
    }
    else if (topic_type[0] == "jog_frame")
    {
      std::vector<std::string> group_names = readYaml("/RTRDoubleArmV7/mani_gipper/group_name/");
      std::vector<std::string> link_names = readYaml("/RTRDoubleArmV7/mani_gipper/link_name/");
      jog_frame_.header.frame_id = "base_footprint";
      jog_frame_.group_name = group_names[0];
      jog_frame_.link_name = link_names[0];
      jog_frame_.linear_delta.x = 0.01 * joy_msg_.axes[7];
      jog_frame_.linear_delta.y = 0.01 * joy_msg_.axes[6];
      jog_frame_.linear_delta.z = 0.01 * (joy_msg_.buttons[5] - joy_msg_.buttons[4]);
      jog_frame_.angular_delta.x = 0.01 * (joy_msg_.buttons[1] - joy_msg_.buttons[3]);
      jog_frame_.angular_delta.y = 0.01 * (joy_msg_.buttons[0] - joy_msg_.buttons[2]);
      jog_frame_.angular_delta.z = 0.00;
      jog_frame_.avoid_collisions = false;
      jog_frame_.header.stamp = ros::Time::now();
      jog_frame_pub_.publish(jog_frame_);
    }

    // joint trajectory for mani_gipper
    trajectory_.points.clear();
    trajectory_.joint_names.clear();
    trajectory_point_.positions.clear();
    for (auto joint_name : mani_gipper_joint_names_)
    {
      trajectory_.joint_names.push_back(joint_name);
      trajectory_point_.positions.push_back(0.05 / 2.0 * (jogCommandSet("PUSHROD") - 1.0));
    }
    ros::Duration time(1.0);
    trajectory_point_.time_from_start = time;
    trajectory_.points.push_back(trajectory_point_);

    // set publisher
    trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/RTRDoubleArmV7/mani_gripper_controller/command", 10);
    trajectory_pub_.publish(trajectory_);
  }
}

// pathに該当するyaml内の値を返すmethod
std::vector<std::string> RTRJoycon::readYaml(const std::string path)
{
  std::vector<std::string> param_list;
  if (!nh_.getParam(path, param_list))
  {
    std::string error_str = "Failed to get parameter[" + path + "] from yaml file.";
    ROS_ERROR_STREAM(error_str);
  }
  return param_list;
}

// yamlに記述されたキー設定をkey_config_にセットするmethod
bool RTRJoycon::setKeyConfig(const int config_num)
{
  key_config_.clear();
  std::string config_path = "/setting_" + std::to_string(config_num) + "/key_config/";
  for (auto key_name : joy_con_key_names_)
  {
    std::string config_str;
    if (!nh_.getParam(config_path + key_name,   config_str)) return false;
    key_config_.push_back(config_str);
  }

  std::cout << "set key_config to config_" << config_num << std::endl;
  return true;
}

float RTRJoycon::jogCommandSet(const std::string joint_name)
{
  int axes_size = joy_msg_.axes.size();

  if (joint_name == "TOHKU_PITCH")
  {
    const std::string TOHKU_PITCH_UP = "TOHKU_PITCH_UP";
    const int TOHKU_PITCH_UP_index = getStringIndex(key_config_, TOHKU_PITCH_UP);
    const std::string TOHKU_PITCH_DOWN = "TOHKU_PITCH_DOWN";
    const int TOHKU_PITCH_DOWN_index = getStringIndex(key_config_, TOHKU_PITCH_DOWN);
    if (TOHKU_PITCH_UP_index < 0 || TOHKU_PITCH_DOWN_index < 0) return 0.0; // 設定されていない場合

    if (TOHKU_PITCH_UP_index < axes_size)
      return joy_msg_.axes[TOHKU_PITCH_UP_index] - joy_msg_.axes[TOHKU_PITCH_DOWN_index];
    else
      return joy_msg_.buttons[TOHKU_PITCH_UP_index-axes_size] - joy_msg_.buttons[TOHKU_PITCH_DOWN_index-axes_size];
  }
  else if (joint_name == "TOHKU_ROLL")
  {
    const std::string TOHKU_ROLL_UP = "TOHKU_ROLL_UP";
    const int TOHKU_ROLL_UP_index = getStringIndex(key_config_, TOHKU_ROLL_UP);
    const std::string TOHKU_ROLL_DOWN = "TOHKU_ROLL_DOWN";
    const int TOHKU_ROLL_DOWN_index = getStringIndex(key_config_, TOHKU_ROLL_DOWN);
    if (TOHKU_ROLL_UP_index < 0 || TOHKU_ROLL_DOWN_index < 0) return 0.0; // 設定されていない場合

    if (TOHKU_ROLL_UP_index < axes_size)
      return joy_msg_.axes[TOHKU_ROLL_UP_index] - joy_msg_.axes[TOHKU_ROLL_DOWN_index];
    else
      return joy_msg_.buttons[TOHKU_ROLL_UP_index-axes_size] - joy_msg_.buttons[TOHKU_ROLL_DOWN_index-axes_size];
  }
  else if (joint_name == "MANIELBOW")
  {
    const std::string MANIELBOW_UP = "MANIELBOW_UP";
    const int MANIELBOW_UP_index = getStringIndex(key_config_, MANIELBOW_UP);
    const std::string MANIELBOW_DOWN = "MANIELBOW_DOWN";
    const int MANIELBOW_DOWN_index = getStringIndex(key_config_, MANIELBOW_DOWN);
    if (MANIELBOW_UP_index < 0 || MANIELBOW_DOWN_index < 0) return 0.0; // 設定されていない場合

    if (MANIELBOW_UP_index < axes_size)
      return joy_msg_.axes[MANIELBOW_UP_index] - joy_msg_.axes[MANIELBOW_DOWN_index];
    else
      return joy_msg_.buttons[MANIELBOW_UP_index-axes_size] - joy_msg_.buttons[MANIELBOW_DOWN_index-axes_size];
  }
  else if (joint_name == "YAWJOINT")
  {
    const std::string YAWJOINT_UP = "YAWJOINT_UP";
    const int YAWJOINT_UP_index = getStringIndex(key_config_, YAWJOINT_UP);
    const std::string YAWJOINT_DOWN = "YAWJOINT_DOWN";
    const int YAWJOINT_DOWN_index = getStringIndex(key_config_, YAWJOINT_DOWN);
    if (YAWJOINT_UP_index < 0 || YAWJOINT_DOWN_index < 0) return 0.0; // 設定されていない場合

    if (YAWJOINT_UP_index < axes_size)
      return joy_msg_.axes[YAWJOINT_UP_index] - joy_msg_.axes[YAWJOINT_DOWN_index];
    else
      return joy_msg_.buttons[YAWJOINT_UP_index-axes_size] - joy_msg_.buttons[YAWJOINT_DOWN_index-axes_size];
  }
  else if (joint_name == "TOHKU_TIP_01" || joint_name == "TOHKU_TIP_02")
  {
    const std::string TOHKU_TIP = "TOHKU_TIP";
    const int TOHKU_TIP_index = getStringIndex(key_config_, TOHKU_TIP);
    if (TOHKU_TIP_index < 0) return 0.0;

    if (TOHKU_TIP_index < axes_size)
      return joy_msg_.axes[TOHKU_TIP_index];
    else
      return joy_msg_.buttons[TOHKU_TIP_index - axes_size];
  }
  else if (joint_name == "PUSHROD")
  {
    const std::string PUSHROD = "PUSHROD";
    const int PUSHROD_index = getStringIndex(key_config_, PUSHROD);
    if (PUSHROD_index < 0) return 0.0;

    if (PUSHROD_index < axes_size)
      return joy_msg_.axes[PUSHROD_index];
    else
      return joy_msg_.buttons[PUSHROD_index-axes_size];
  }
  else
  {
    const int index = getStringIndex(key_config_, joint_name);
    if (index < 0) return 0.0;

    if (index < axes_size)
      return joy_msg_.axes[index];
    else
      return joy_msg_.buttons[index-axes_size];
  }
}

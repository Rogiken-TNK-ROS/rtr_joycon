#ifndef RTR_JOY_CON_H
#define RTR_JOY_CON_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <jog_msgs/JogJoint.h>
#include <jog_msgs/JogFrame.h>
#include <trajectory_msgs/JointTrajectory.h>

class RtrJoyconState
{
public:
  RtrJoyconState(ros::NodeHandle& nh);
  void publish(void);

private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_node_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher jog_joint_pub_;
  ros::Publisher jog_frame_pub_;
  ros::Publisher tohoku_trajectory_pub_;
  ros::Publisher mani_trajectory_pub_;

  sensor_msgs::Joy joy_msg_;
  sensor_msgs::Joy joy_msg_buf_;

  std::vector<std::string> settings_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> group_names_;
  std::vector<std::string> link_names_;
  std::vector<std::string> tohoku_gipper_joint_names_;
  std::vector<std::string> mani_gipper_joint_names_;
  float speed_rate_;
  int config_num_;

  using JoyConfigType = std::function<void(std::string const&)>;
  JoyConfigType joy_config_;

private:
  void updateJoyMsg(const sensor_msgs::Joy& joy_msg);
  float jogCommandSet(const std::string joint_name);
  bool getStringIndex(int &index, const std::vector<std::string> str_vec, const std::string string);
  void readYaml(const std::string config);
  void updateState(void);

  JoyConfigType config_0()
  {
    return [=](std::string const& s) 
    {
      std::cout << s << "0" << std::endl;
    };
  }
  JoyConfigType config_1()
  {
    return [=](std::string const& s)
    {
      std::cout << s << "1" << std::endl;
    };
  }
  JoyConfigType config_2()
  {
    return [=](std::string const& s)
    {
      std::cout << s << "2" << std::endl;
    };
  }

};

#endif  // RTR_JOY_CON_H
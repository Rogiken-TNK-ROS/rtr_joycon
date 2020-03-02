#ifndef RTR_JOY_CON_H
#define RTR_JOY_CON_H

#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class RtrJoyconState
{
public:
  RtrJoyconState(ros::NodeHandle& nh);
  void publish(void);
  void WriteString(std::string const& s);

private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_node_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher arm_vel_pub_;

  geometry_msgs::Twist cmd_vel_;
  control_msgs::JointTrajectoryControllerState arm_vel_;
  sensor_msgs::Joy joy_msg_;
  sensor_msgs::Joy joy_msg_buf_;

  std::vector<std::string> axes_;
  std::vector<std::string> buttons_;
  float speed_rate_;
  int config_num_;

  using JoyConfigType = std::function<void(std::string const&)>;
  JoyConfigType joy_config_;

private:
  void updateJoyMsg(const sensor_msgs::Joy& joy_msg);
  int getStringIndex(const std::string string);
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
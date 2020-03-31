#ifndef RTR_JOYCON_H
#define RTR_JOYCON_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <jog_msgs/JogJoint.h>
#include <jog_msgs/JogFrame.h>
#include <trajectory_msgs/JointTrajectory.h>

class RTRJoycon
{
public:
  RTRJoycon(ros::NodeHandle& nh);
  void publish(void);

private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_cmd_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher jog_joint_pub_;
  ros::Publisher tohoku_trajectory_pub_;
  ros::Publisher mani_trajectory_pub_;

  sensor_msgs::Joy joy_msg_;
  std::vector<std::string> key_config_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> tohoku_gipper_joint_names_;
  std::vector<std::string> mani_gipper_joint_names_;

  float speed_rate_;
  int config_num_;

private:
  void updateJoyMsg(const sensor_msgs::Joy& joy_msg);
  void readYaml(const std::string config);
};

#endif  // RTR_JOYCON_H
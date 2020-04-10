#ifndef RTR_JOYCON_H
#define RTR_JOYCON_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <jog_msgs/JogJoint.h>
#include <jog_msgs/JogFrame.h>

class RTRJoycon
{
public:
  RTRJoycon(ros::NodeHandle& nh);
  void publish(void); // 実行関数
  void updateCommand(void);

private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_cmd_sub_;   // joy_node用subscriber
  ros::Publisher cmd_vel_pub_;    // command_vel用publisher
  ros::Publisher jog_joint_pub_;  // jog_joint用publisher
  ros::Publisher jog_frame_pub_;  // jog_frame用publisher
  ros::Publisher trajectory_pub_;    // JointTrajectory用publisher

  ros::Publisher L1_enable_pub_;
  ros::Publisher R1_enable_pub_;
  ros::Publisher mani_arm_enable_pub_;

  sensor_msgs::Joy joy_msg_;  // joy_nodeコマンド格納用

  std::vector<std::string> double_arm_joint_names_;     // yamlファイルから読み取ったdouble_armのjoint_name
  std::vector<std::string> tohoku_gipper_joint_names_;  // yamlファイルから読み取ったtohoku_gipperのjoint_name
  std::vector<std::string> mani_gipper_joint_names_;    // yamlファイルから読み取ったmani_gipperのjoint_name

  float speed_gain_;    // 速度ゲイン

  std_msgs::Bool L1_enable_;
  std_msgs::Bool R1_enable_;
  std_msgs::Bool mani_arm_enable_;

  geometry_msgs::Twist cmd_vel_;  // 指令値[タイヤ]
  jog_msgs::JogJoint jog_joint_;  // 指令値[ウデ]
  jog_msgs::JogFrame jog_frame_;  // 指令値[ウデ]
  trajectory_msgs::JointTrajectory trajectory_;             // 指令値[ハンド]
  trajectory_msgs::JointTrajectoryPoint trajectory_point_;  // 指令値[ハンド]

private:
  void updateJoyMsg(const sensor_msgs::Joy& joy_msg);           // メンバ変数joy_msg_の更新関数（callback）
  bool setCommand(std::string task, float positive, float negative);

  template<typename T>
  int getIndexFromVector(std::vector<T> vec, const T target)
  {
    typename std::vector<T>::iterator itr;
    itr = std::find(vec.begin(), vec.end(), target);
    if (itr == vec.end()) return -1;
    const int index = std::distance(vec.begin(), itr);
    return index;
  }

  template<typename TYPE>
  TYPE readYaml(const std::string path)
  {
    TYPE param;
    if (!nh_.getParam(path, param))
    {
      std::string error_str = "Failed to get parameter[" + path + "] from yaml file.";
      ROS_ERROR_STREAM(error_str);
    }
    return param;
  }

};

#endif  // RTR_JOYCON_
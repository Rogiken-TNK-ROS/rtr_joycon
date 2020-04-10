#ifndef RTR_JOYCON_H
#define RTR_JOYCON_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <jog_msgs/JogJoint.h>
#include <jog_msgs/JogFrame.h>

class RTRJoycon
{
public:
  RTRJoycon(ros::NodeHandle& nh);
  void publish(void); // 実行関数

private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_cmd_sub_;   // joy_node用subscriber
  ros::Publisher cmd_vel_pub_;    // command_vel用publisher
  ros::Publisher jog_joint_pub_;  // jog_joint用publisher
  ros::Publisher jog_frame_pub_;  // jog_frame用publisher
  ros::Publisher trajectory_pub_;    // JointTrajectory用publisher

  sensor_msgs::Joy joy_msg_;  // joy_nodeコマンド格納用

  std::vector<std::string> joy_con_key_names_;
  std::vector<std::string> double_arm_joint_names_;     // yamlファイルから読み取ったdouble_armのjoint_name
  std::vector<std::string> tohoku_gipper_joint_names_;  // yamlファイルから読み取ったtohoku_gipperのjoint_name
  std::vector<std::string> mani_gipper_joint_names_;    // yamlファイルから読み取ったmani_gipperのjoint_name

  const float speed_gain_ = 0.2;    // 速度ゲイン
  int key_config_index_;            // キー設定のindex
  std::vector<std::string> key_config_;   // yamlファイルから読み取ったキー設定

  geometry_msgs::Twist cmd_vel_;  // 指令値[タイヤ]
  jog_msgs::JogJoint jog_joint_;  // 指令値[ウデ]
  jog_msgs::JogFrame jog_frame_;  // 指令値[ウデ]
  trajectory_msgs::JointTrajectory trajectory_;             // 指令値[ハンド]
  trajectory_msgs::JointTrajectoryPoint trajectory_point_;  // 指令値[ハンド]

private:
  void updateJoyMsg(const sensor_msgs::Joy& joy_msg);           // メンバ変数joy_msg_の更新関数（callback）
  std::vector<std::string> readYaml(const std::string path);    // pathに該当するyaml内の値を返す関数
  bool setKeyConfig(const int config_num);                      // yamlに記述されたキー設定をkey_config_にセットするmethod
  float jogCommandSet(const std::string joint_name);

  template<typename T>
  int getIndexFromVector(std::vector<T> vec, const T target)
  {
    typename std::vector<T>::iterator itr;
    itr = std::find(vec.begin(), vec.end(), target);
    if (itr == vec.end()) return -1;
    const int index = std::distance(vec.begin(), itr);
    return index;
  }

};

#endif  // RTR_JOYCON_H
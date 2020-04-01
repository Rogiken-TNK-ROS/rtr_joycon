#ifndef RTR_JOYCON_H
#define RTR_JOYCON_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
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
  ros::Publisher jog_frame_pub_;  // jog_frame用publisher

  sensor_msgs::Joy joy_msg_;  // joy_nodeコマンド格納用

  std::vector<std::string> double_arm_joint_names_;     // yamlファイルから読み取ったdouble_armのjoint_name
  std::vector<std::string> tohoku_gipper_joint_names_;  // yamlファイルから読み取ったtohoku_gipperのjoint_name
  std::vector<std::string> mani_gipper_joint_names_;    // yamlファイルから読み取ったmani_gipperのjoint_name

  const float speed_rate_ = 0.2;    // 速度ゲイン
  int key_config_index_;            // 設定されているキー設定index
  std::vector<std::string> key_config_;   // yamlファイルから読み取ったキー設定

private:
  void updateJoyMsg(const sensor_msgs::Joy& joy_msg);           // メンバ変数joy_msg_の更新関数（callback）
  std::vector<std::string> readYaml(const std::string path);    // pathに該当するyaml内の値を返す関数
  bool getKeyConfig(const int config_num);                      // キーコンフィグを取得する関数
  bool getStringIndex(int &index, const std::vector<std::string> str_vec, const std::string string);
};

#endif  // RTR_JOYCON_H
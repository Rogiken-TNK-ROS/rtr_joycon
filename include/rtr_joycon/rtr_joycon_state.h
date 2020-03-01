#ifndef RTR_JOY_CON_H
#define RTR_JOY_CON_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <functional>
#include <string>
#include <memory>
#include <iostream>

class RtrJoyconState
{
public:
  RtrJoyconState(ros::NodeHandle& nh);
  void WriteString(std::string const& s);

private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_node_sub_;
  int count_;

  using JoyConfigType = std::function<void(std::string const&)>;
  JoyConfigType joy_config_;

private:
  void updateState(const sensor_msgs::Joy& joy_msg);

  JoyConfigType config_1()
  {
    return [=](std::string const& s) 
    {
      std::cout << s << std::endl;
    };
  }
  
  JoyConfigType config_2()
  {
    return [=](std::string const& s)
    {
      std::cout << s << "+" << std::endl;
    };
  }

};

#endif  // RTR_JOY_CON_H
#ifndef RTR_JOYCON_KEYMAP_CONFIG_H_
#define RTR_JOYCON_KEYMAP_CONFIG_H_

#include <ros/ros.h>

namespace rtr
{
class KeymapConfig
{
 public:
  KeymapConfig() = default;
  void init(const ros::NodeHandle& nh);

  const int map(const std::string& key) const { return map_.at(key); };
  
 private:
  std::map<std::string, int> map_;
};
}


#endif // RTR_JOYCON_KEYMAP_CONFIG_H_

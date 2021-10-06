#include <rtr_joycon/keymap_config.h>

namespace rtr {
void KeymapConfig::init(const ros::NodeHandle& nh)
{
   // axes
  int idx;
  nh.getParam("axes/L_stick_H", idx);
  map_["axes/L_stick_H"] = idx;
  nh.getParam("axes/L_stick_V", idx);
  map_["axes/L_stick_V"] = idx;
  nh.getParam("axes/R_stick_H", idx);
  map_["axes/R_stick_H"] = idx;
  nh.getParam("axes/R_stick_V", idx);
  map_["axes/R_stick_V"] = idx;
  nh.getParam("axes/L2_trigger", idx);
  map_["axes/L2_trigger"] = idx;
  nh.getParam("axes/R2_trigger", idx);
  map_["axes/R2_trigger"] = idx;
  nh.getParam("axes/cross_key_H", idx);
  map_["axes/cross_key_H"] = idx;
  nh.getParam("axes/cross_key_V", idx);
  map_["axes/cross_key_V"] = idx;
  // buttons
  nh.getParam("buttons/A_button", idx);
  map_["buttons/A_button"] = idx;
  nh.getParam("buttons/B_button", idx);
  map_["buttons/B_button"] = idx;
  nh.getParam("buttons/X_button", idx);
  map_["buttons/X_button"] = idx;
  nh.getParam("buttons/Y_button", idx);
  map_["buttons/Y_button"] = idx;
  nh.getParam("buttons/L1_button", idx);
  map_["buttons/L1_button"] = idx;
  nh.getParam("buttons/R1_button", idx);
  map_["buttons/R1_button"] = idx;
  nh.getParam("buttons/L2_button", idx);
  map_["buttons/L2_button"] = idx;
  nh.getParam("buttons/R2_button", idx);
  map_["buttons/R2_button"] = idx;
  nh.getParam("buttons/L3_button", idx);
  map_["buttons/L3_button"] = idx;
  nh.getParam("buttons/R3_button", idx);
  map_["buttons/R3_button"] = idx;
  nh.getParam("buttons/option_button", idx);
  map_["buttons/option_button"] = idx;
  nh.getParam("buttons/start_button", idx);
  map_["buttons/start_button"] = idx;
  nh.getParam("buttons/home_button", idx);
  map_["buttons/home_button"] = idx;
  nh.getParam("buttons/cross_U", idx);
  map_["buttons/cross_U"] = idx;
  nh.getParam("buttons/cross_D", idx);
  map_["buttons/cross_D"] = idx;
  nh.getParam("buttons/cross_L", idx);
  map_["buttons/cross_L"] = idx;
  nh.getParam("buttons/cross_R", idx);
  map_["buttons/cross_R"] = idx;
}

}

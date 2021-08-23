#include <rtr_joycon/tohoku_ik_control.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rtr
{
TohokuIKControl::TohokuIKControl()
{
  using namespace std;
  using namespace moveit::planning_interface;
  
  mi_ = make_shared<MoveGroupInterface>(group_);
  psi_ = make_shared<PlanningSceneInterface>();

  ros::NodeHandle nh("~");
  kc_.init(nh);
  
  std::string str;
  nh.getParam("TOHOKU_IK/RPY_mode",str);
  rpy_mode_button_ = kc_.map(str);
  nh.getParam("TOHOKU_IK/X/PLUS",str);
  keymap_["X_PLUS"] = str;
  nh.getParam("TOHOKU_IK/Y/PLUS",str);
  keymap_["Y_PLUS"] = str;
  nh.getParam("TOHOKU_IK/Z/PLUS",str);
  keymap_["Z_PLUS"] = str;
  nh.getParam("TOHOKU_IK/ROLL/PLUS",str);
  keymap_["ROLL_PLUS"] = str;
  nh.getParam("TOHOKU_IK/PITCH/PLUS",str);
  keymap_["PITCH_PLUS"] = str;
  nh.getParam("TOHOKU_IK/YAW/PLUS",str);
  keymap_["YAW_PLUS"] = str;
  nh.getParam("TOHOKU_IK/X/MINUS",str);
  keymap_["X_MINUS"] = str;
  nh.getParam("TOHOKU_IK/Y/MINUS",str);
  keymap_["Y_MINUS"] = str;
  nh.getParam("TOHOKU_IK/Z/MINUS",str);
  keymap_["Z_MINUS"] = str;
  nh.getParam("TOHOKU_IK/ROLL/MINUS",str);
  keymap_["ROLL_MINUS"] = str;
  nh.getParam("TOHOKU_IK/PITCH/MINUS",str);
  keymap_["PITCH_MINUS"] = str;
  nh.getParam("TOHOKU_IK/YAW/MINUS",str);
  keymap_["YAW_MINUS"] = str;

  nh.getParam("TOHOKU_IK/axis_threshold",axis_threshold_);
  nh.getParam("TOHOKU_IK/slow_step_xyz",slow_step_xyz_);
  nh.getParam("TOHOKU_IK/slow_step_rpy",slow_step_rpy_);
  nh.getParam("TOHOKU_IK/fast_step_xyz",fast_step_xyz_);
  nh.getParam("TOHOKU_IK/fast_step_rpy",fast_step_rpy_);
}

void TohokuIKControl::control(const sensor_msgs::Joy& joy, const double& xyz_step, const double& rpy_step)
{
  geometry_msgs::PoseStamped current_pose = mi_->getCurrentPose("TOHKU_PITCH");
  geometry_msgs::Pose goal_pose = current_pose.pose;

  for(auto i=keymap_.begin(); i!=keymap_.end(); i++) {
    double plus_value = 0;
    double minus_value = 0;
    if(!joy.buttons[rpy_mode_button_]) {
      if(i->second.substr(0,4) == "axis") {
        plus_value = xyz_step * (joy.axes[kc_.map(i->second)] >= axis_threshold_);
        minus_value = -xyz_step * (joy.axes[kc_.map(i->second)] <= -axis_threshold_);
      } else if(i->second.substr(0,5) == "buttons") {
        plus_value = xyz_step * joy.buttons[kc_.map(i->second)];
        minus_value = -xyz_step * joy.buttons[kc_.map(i->second)];
      } else {
        plus_value = 0;
        minus_value = 0;
      }
      if(i->first.substr(0,1) == "X")
        goal_pose.position.x += plus_value + minus_value;
      else if(i->first.substr(0,1) == "Y")
        goal_pose.position.y += plus_value + minus_value;
      else if(i->first.substr(0,1) == "Z") 
        goal_pose.position.z += plus_value + minus_value;
      else
        goal_pose.position = current_pose.pose.position;
    } else {
      if(i->second.substr(0,4) == "axis") {
        plus_value = rpy_step * (joy.axes[kc_.map(i->second)] >= axis_threshold_);
        minus_value = -rpy_step * (joy.axes[kc_.map(i->second)] <= -axis_threshold_);
      } else if(i->second.substr(0,5) == "buttons") {
        plus_value = rpy_step * joy.buttons[kc_.map(i->second)];
        minus_value = -rpy_step * joy.buttons[kc_.map(i->second)];
      } else {
        plus_value = 0;
        minus_value = 0;
      }
      tf2::Quaternion rotation;
      rotation.setValue(current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w);
      tf2::Matrix3x3 m(rotation);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      if(i->first.substr(0,4) == "ROLL")
        roll += plus_value + minus_value;
      else if(i->first.substr(0,5) == "PITCH")
        pitch += plus_value + minus_value;
      else if(i->first.substr(0,3) == "YAW")
        yaw += plus_value + minus_value;
      else ;
      rotation.setRPY(roll,pitch,yaw);
      rotation.normalize();
      goal_pose.orientation.x = rotation.x();
      goal_pose.orientation.y = rotation.y();
      goal_pose.orientation.z = rotation.z();
      goal_pose.orientation.w = rotation.w();
    }
  }

  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.emplace_back(current_pose.pose);
  waypoints.emplace_back(goal_pose);
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;
  double fraction = mi_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  mi_->execute(trajectory);
}

void TohokuIKControl::slowControl(const sensor_msgs::Joy& joy)
{
  control(joy, slow_step_xyz_, slow_step_rpy_);
}


void TohokuIKControl::fastControl(const sensor_msgs::Joy& joy)
{
  control(joy, fast_step_xyz_, fast_step_rpy_);
}

} // namespace rtr

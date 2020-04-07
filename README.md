# rtr_joycon

## Description
This package converts commands from joy_node into command values for the RTRDoubleArmV7 robot.

## Usage
### Topic I/O
#### subscribe
  - joy

#### publish
  - cmd_vel
  - jog_joint
  - jog_frame
  - JointTrajectory

### Config files
  - rtr_double_arm_joints.yaml
    - Joint information for RTRDoubleArmV7
  - PS4_key_names.yaml
    - Button names for the Dual Shock 4 (PS4)
  - PS4.yaml
    - key configuration file (configurable)
    - 
# rtr_joycon

## Description
This package converts commands from joy_node into command values for the RTRDoubleArmV7 robot.

## Usage
### Topic I/O
#### subscribe
  - joy

#### publish
  - cmd_vel (for moving)
  - jog_joint or jog_frame (for arm)
  - JointTrajectory (for gripper)

### Config files
  - PS4.yaml (**configurable**)
    - key configuration file
      - setting_0 : robot movement
      - setting_1 : work of tohoku_arm (using jog_joint)
      - setting_2 : work of tohoku_arm (using jog_frame)
      - setting_3 : work of mani_arm (using jog_joint)
      - setting_4 : work of mani_arm (using jog_frame)
  - rtr_double_arm_joints.yaml (**unconfigurable**)
    - Joint information for RTRDoubleArmV7
  - PS4_key_names.yaml (**unconfigurable**)
    - Button names for the Dual Shock 4 (PS4)

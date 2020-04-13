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
    - [key map](https://github.com/calm0815/rtr_joycon/blob/master/config/memo.md)
    - This file is key configuration file  
    **Note** : You can switch the settings by pressing the option button

  | コントローラ | ロボット動作 |
  | ---- | ---- |
  | 方向キー	                            | クローラ
  | L1ボタン + 左スティック               | クローラ
  | L1ボタン + 右スティック               | クローラ
  | 左スティック横軸	                    | アームベース ヨー軸
  | 右スティック横軸	                    | アーム第一関節 ヨー軸
  | 左スティック縦軸	                    | アーム第一関節 ピッチ軸
  | 右スティック縦軸	                    | アーム第二関節 ピッチ軸
  | Y、Aボタン（△、×ボタン）            | エンドエフェクタ ピッチ軸
  | X、Bボタン（□、○ボタン）            | エンドエフェクタ ヨー軸
  | L1ボタン + X、Bボタン（□、○ボタン）	| エンドエフェクタ ロール軸
  | R1ボタン、R2トリガ	                  | エンドエフェクタ 開閉
  | optionボタン	                        | 操作対象アームの切り替え

  - PS4_usb_keymap.yaml (**configurable**)
    - Key map for the Dual Shock 4 (usb connected)
  - rtr_double_arm_joints.yaml (**unconfigurable**)
    - Joint information for RTRDoubleArmV7

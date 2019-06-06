<title>Se3Ctrl Component Dictionary</title>
# Se3Ctrl Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|SE3CTRL_SetPosCtrlMode|0 (0x0)|Set position controller mode| | |
| | | |mode|PosCtrlMode||
|SE3CTRL_SetAttCtrlMode|1 (0x1)|Set attitude controller mode| | |
| | | |mode|AttCtrlMode||
|SE3CTRL_LinearSetpoint|2 (0x2)|| | |
| | | |x_w__x|F64||
| | | |x_w__y|F64||
| | | |x_w__z|F64||
| | | |v_w__x|F64||
| | | |v_w__y|F64||
| | | |v_w__z|F64||
|SE3CTRL_AngularSetpoint|3 (0x3)|| | |
| | | |w_q_b__x|F64||
| | | |w_q_b__y|F64||
| | | |w_q_b__z|F64||
| | | |w_q_b__w|F64||
| | | |omega_b__x|F64||
| | | |omega_b__y|F64||
| | | |omega_b__z|F64||
|SE3CTRL_InitParams|4 (0x4)|| | |

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|SE3CTRL_MomentCommX|1 (0x1)|F32|SE3 Control commanded moment in x|
|SE3CTRL_MomentCommY|2 (0x2)|F32|SE3 Control commanded moment in y|
|SE3CTRL_MomentCommZ|3 (0x3)|F32|SE3 Control commanded moment in z|
|SE3CTRL_Error_x_w|4 (0x4)|F32|SE3 Control x error|
|SE3CTRL_Error_y_w|5 (0x5)|F32|SE3 Control y error|
|SE3CTRL_Error_z_w|6 (0x6)|F32|SE3 Control z error|
|SE3CTRL_XThrustComm|7 (0x7)|F32|SE3 Control commanded thrust in x direction|
|SE3CTRL_YThrustComm|8 (0x8)|F32|SE3 Control commanded thrust in x direction|
|SE3CTRL_ZThrustComm|9 (0x9)|F32|SE3 Control commanded thrust in z direction|
|SE3CTRL_w_q_b__des|10 (0xa)|ROS::geometry_msgs::Quaternion|SE3 Control desired orientation|
|SE3CTRL_w_q_b|11 (0xb)|ROS::geometry_msgs::Quaternion|SE3 Control orientation|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|SE3CTRL_Dummy|0 (0x0)|SE3 Control dummy event| | | | |

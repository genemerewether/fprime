<title>ActuatorAdapter Component Dictionary</title>
# ActuatorAdapter Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|ACTADAP_Arm|0 (0x0)|| | |   
| | | |armState|bool||                    
|ACTADAP_InitParams|1 (0x1)|| | |   
|ACTADAP_SetVoltAct|2 (0x2)|| | |   
| | | |actIdx|U8||                    
| | | |voltage|F64||                    

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|ACTADAP_Rot0|0 (0x0)|F32||
|ACTADAP_Rot1|1 (0x1)|F32||
|ACTADAP_Rot2|2 (0x2)|F32||
|ACTADAP_Rot3|3 (0x3)|F32||
|ACTADAP_Rot4|4 (0x4)|F32||
|ACTADAP_Rot5|5 (0x5)|F32||
|ACTADAP_Rot6|6 (0x6)|F32||
|ACTADAP_Rot7|7 (0x7)|F32||
|ACTADAP_Cmd0|10 (0xa)|I32||
|ACTADAP_Cmd1|11 (0xb)|I32||
|ACTADAP_Cmd2|12 (0xc)|I32||
|ACTADAP_Cmd3|13 (0xd)|I32||
|ACTADAP_Cmd4|14 (0xe)|I32||
|ACTADAP_Cmd5|15 (0xf)|I32||
|ACTADAP_Cmd6|16 (0x10)|I32||
|ACTADAP_Cmd7|17 (0x11)|I32||
|ACTADAP_CmdVel0|20 (0x14)|F32||
|ACTADAP_CmdVel1|21 (0x15)|F32||
|ACTADAP_CmdVel2|22 (0x16)|F32||
|ACTADAP_CmdVel3|23 (0x17)|F32||
|ACTADAP_CmdVel4|24 (0x18)|F32||
|ACTADAP_CmdVel5|25 (0x19)|F32||
|ACTADAP_CmdVel6|26 (0x1a)|F32||
|ACTADAP_CmdVel7|27 (0x1b)|F32||
|ACTADAP_ArmState|30 (0x1e)|ArmStateTlm||
|ACTADAP_HwEnabled|31 (0x1f)|bool||

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|ACTADAP_AlreadyArmed|0 (0x0)|| | | | |
|ACTADAP_Error|1 (0x1)|| | | | |
| | | |error|ErrorType||The error code|    
|ACTADAP_NotFlySafe|2 (0x2)|| | | | |
| | | |reason|NotFlySafeType||The error code|    

<title>ActuatorAdapter Component Dictionary</title>
# ActuatorAdapter Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|ACTADAP_Arm|0 (0x0)|| | |   
| | | |armState|bool||                    
|ACTADAP_InitParams|1 (0x1)|| | |   

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|ACTADAP_Rot0|0 (0x0)|F32||
|ACTADAP_Rot1|1 (0x1)|F32||
|ACTADAP_Rot2|2 (0x2)|F32||
|ACTADAP_Rot3|3 (0x3)|F32||
|ACTADAP_Rot4|4 (0x4)|F32||
|ACTADAP_Rot5|5 (0x5)|F32||
|ACTADAP_Cmd0|10 (0xa)|U32||
|ACTADAP_Cmd1|11 (0xb)|U32||
|ACTADAP_Cmd2|12 (0xc)|U32||
|ACTADAP_Cmd3|13 (0xd)|U32||
|ACTADAP_Cmd4|14 (0xe)|U32||
|ACTADAP_Cmd5|15 (0xf)|U32||
|ACTADAP_CmdVel0|20 (0x14)|F32||
|ACTADAP_CmdVel1|21 (0x15)|F32||
|ACTADAP_CmdVel2|22 (0x16)|F32||
|ACTADAP_CmdVel3|23 (0x17)|F32||
|ACTADAP_CmdVel4|24 (0x18)|F32||
|ACTADAP_CmdVel5|25 (0x19)|F32||

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|ACTADAP_AlreadyArmed|0 (0x0)|| | | | |

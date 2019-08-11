<title>QualcommESC Component Dictionary</title>
# QualcommESC Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|QCESC_InitParams|0 (0x0)|| | |
|QCESC_Arm|1 (0x1)|| | |
| | | |armState|bool||

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|QCESC_Rot0|0 (0x0)|F32||
|QCESC_Rot1|1 (0x1)|F32||
|QCESC_Rot2|2 (0x2)|F32||
|QCESC_Rot3|3 (0x3)|F32||
|QCESC_Rot4|4 (0x4)|F32||
|QCESC_Rot5|5 (0x5)|F32||
|QCESC_Rot6|6 (0x6)|F32||
|QCESC_Rot7|7 (0x7)|F32||

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|QCESC_InitFailed|0 (0x0)|QCESC init failed for the following reason| | | | |
| | | |error|QCESC_InitErrorType||The error code|

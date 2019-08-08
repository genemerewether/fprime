<title>ActuatorControls Component Dictionary</title>
# ActuatorControls Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|ACTCTRL_InitParams|0 (0x0)|| | |

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|ACTCTRL_ThrustComm|0 (0x0)|F32|Mavlink commanded body axis thrust|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|ACTCTRL_InitFailed|0 (0x0)|ACTCTRL init failed for the following reason| | | | |
| | | |error|ACTCTRL_InitErrorType||The error code|

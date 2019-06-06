<title>ATINetbox Component Dictionary</title>
# ATINetbox Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|ATINET_BIAS|0 (0x0)|| | |
|ATINET_ENABLE|1 (0x1)|| | |
| | | |mode|ATINetboxEnableMode||
|ATINET_DEBUG|2 (0x2)|| | |
|ATINET_TARE|3 (0x3)|| | |
| | | |cycles|U32||
|ATINET_FLOW|4 (0x4)|| | |
| | | |mode|ATINetboxFlowMode||

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|ATINET_ForceX|0 (0x0)|F32||
|ATINET_ForceY|1 (0x1)|F32||
|ATINET_ForceZ|2 (0x2)|F32||
|ATINET_TorqueX|3 (0x3)|F32||
|ATINET_TorqueY|4 (0x4)|F32||
|ATINET_TorqueZ|5 (0x5)|F32||
|ATINET_ForceXRaw|6 (0x6)|F32||
|ATINET_ForceYRaw|7 (0x7)|F32||
|ATINET_ForceZRaw|8 (0x8)|F32||
|ATINET_TorqueXRaw|9 (0x9)|F32||
|ATINET_TorqueYRaw|10 (0xa)|F32||
|ATINET_TorqueZRaw|11 (0xb)|F32||
|ATINET_Health|12 (0xc)|U32||
|ATINET_Packets|13 (0xd)|U32||

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|ATINET_WorkerStarted|0 (0x0)|| | | | |
|ATINET_WorkerStopped|1 (0x1)|| | | | |
|ATINET_SocketCreateFail|2 (0x2)|| | | | |
|ATINET_SocketBindFail|3 (0x3)|| | | | |
|ATINET_SocketCreated|4 (0x4)|| | | | |
|ATINET_Biased|5 (0x5)|| | | | |
|ATINET_Msg|6 (0x6)|| | | | |
| | | |Msg|Fw::LogStringArg&|40|general string message|
|ATINET_Err_Msg|7 (0x7)|| | | | |
| | | |Msg|Fw::LogStringArg&|40|error string message|
|ATINET_PrmLdErr|8 (0x8)|| | | | |
| | | |id|U32||ID of parameter|
|ATINET_PrmUpdated|9 (0x9)|| | | | |
| | | |id|U32||ID of parameter|
|ATINET_TareStarted|10 (0xa)|| | | | |
| | | |cycles|U32||force x tare|
| | | |fx|F32||force x tare|
| | | |fy|F32||force y tare|
| | | |fz|F32||force z tare|
| | | |tx|F32||torque x tare|
| | | |ty|F32||torque y tare|
| | | |tz|F32||torque xz tare|
|ATINET_TareComplete|11 (0xb)|| | | | |
| | | |fx|F32||force x tare|
| | | |fy|F32||force y tare|
| | | |fz|F32||force z tare|
| | | |tx|F32||torque x tare|
| | | |ty|F32||torque y tare|
| | | |tz|F32||torque xz tare|

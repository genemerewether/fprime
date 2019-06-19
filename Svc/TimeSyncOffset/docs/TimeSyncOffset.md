<title>TimeSyncOffset Component Dictionary</title>
# TimeSyncOffset Component Dictionary


## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|LLTime|0 (0x0)|F64|low level time tlm in seconds|
|HLTime|1 (0x1)|F64|high level time tlm in seconds|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|SchedIn_Timeout|0 (0x0)|event warning when no LL time is recieved after timeout num of sched calls| | | | |
| | | |sched_timeout|U8|||

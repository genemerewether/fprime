<title>SnapdragonHealth Component Dictionary</title>
# SnapdragonHealth Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|SnapdragonHealth_SetPowerSaver|0 (0x0)|Turn on/off power-saving features| | |
| | | |Mode|PowerSaverMode||

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|SnapTempZone0DspCpuStat|0 (0x0)|bool|The status of temperature zone 0, enabled = 1, disabled = 0.|
|SnapTempZone0DspCpuTemp|1 (0x1)|F32|The temperature of temperature zone 0|
|SnapTempZone1DspModemStat|2 (0x2)|bool|The status of temperature zone 1, enabled = 1, disabled = 0.|
|SnapTempZone1DspModemTemp|3 (0x3)|F32|The temperature of temperature zone 1|
|SnapTempZone2ModemStat|4 (0x4)|bool|The status of temperature zone 2, enabled = 1, disabled = 0.|
|SnapTempZone2ModemTemp|5 (0x5)|F32|The temperature of temperature zone 2|
|SnapTempZone3VideoStat|6 (0x6)|bool|The status of temperature zone 3, enabled = 1, disabled = 0.|
|SnapTempZone3VideoTemp|7 (0x7)|F32|The temperature of temperature zone 3|
|SnapTempZone4WlanCpuStat|8 (0x8)|bool|The status of temperature zone 4, enabled = 1, disabled = 0.|
|SnapTempZone4WlanCpuTemp|9 (0x9)|F32|The temperature of temperature zone 4|
|SnapTempZone5Cpu0Stat|10 (0xa)|bool|The status of temperature zone 5, enabled = 1, disabled = 0.|
|SnapTempZone5Cpu0Temp|11 (0xb)|F32|The temperature of temperature zone 5|
|SnapTempZone6Cpu1Stat|12 (0xc)|bool|The status of temperature zone 6, enabled = 1, disabled = 0.|
|SnapTempZone6Cpu1Temp|13 (0xd)|F32|The temperature of temperature zone 6|
|SnapTempZone7Cpu2Stat|14 (0xe)|bool|The status of temperature zone 7, enabled = 1, disabled = 0.|
|SnapTempZone7Cpu2Temp|15 (0xf)|F32|The temperature of temperature zone 7|
|SnapTempZone8Cpu3Stat|16 (0x10)|bool|The status of temperature zone 8, enabled = 1, disabled = 0.|
|SnapTempZone8Cpu3Temp|17 (0x11)|F32|The temperature of temperature zone 8|
|SnapTempZone9GpuAStat|18 (0x12)|bool|The status of temperature zone 9, enabled = 1, disabled = 0.|
|SnapTempZone9GpuATemp|19 (0x13)|F32|The temperature of temperature zone 9|
|SnapTempZone10GpuBStat|20 (0x14)|bool|The status of temperature zone 10, enabled = 1, disabled = 0.|
|SnapTempZone10GpuBTemp|21 (0x15)|F32|The temperature of temperature zone 10|
|SnapdragonTotalCpuUtil|22 (0x16)|F32|The total CPU utilization percentage|
|SnapdragonCpu0Util|23 (0x17)|F32|The total CPU0 utilization percentage|
|SnapdragonCpu1Util|24 (0x18)|F32|The total CPU1 utilization percentage|
|SnapdragonCpu2Util|25 (0x19)|F32|The total CPU2 utilization percentage|
|SnapdragonCpu3Util|26 (0x1a)|F32|The total CPU3 utilization percentage|
|SnapdragonCpu0Freq|27 (0x1b)|U32|The CPU0 scaling frequency|
|SnapdragonCpu1Freq|28 (0x1c)|U32|The CPU1 scaling frequency|
|SnapdragonCpu2Freq|29 (0x1d)|U32|The CPU2 scaling frequency|
|SnapdragonCpu3Freq|30 (0x1e)|U32|The CPU3 scaling frequency|
|SnapdragonPowerSaver|31 (0x1f)|PowerSaverModeTlm|Power saver state|
|SnapdragonBootCount|32 (0x20)|U32|Boot count variable|
|SnapTempZone11PM8841Stat|33 (0x21)|bool|The status of temperature zone 11, enabled = 1, disabled = 0.|
|SnapTempZone11PM8841Temp|34 (0x22)|F32|The temperature of temperature zone 11|
|SnapTempZone12PM8941Stat|35 (0x23)|bool|The status of temperature zone 12, enabled = 1, disabled = 0.|
|SnapTempZone12PM8941Temp|36 (0x24)|F32|The temperature of temperature zone 12|
|SnapTempZone15EmmcStat|41 (0x29)|bool|The status of temperature zone 15, enabled = 1, disabled = 0.|
|SnapTempZone15EmmcTemp|42 (0x2a)|F32|The temperature of temperature zone 15|
|SnapTempZone16MsmStat|43 (0x2b)|bool|The status of temperature zone 16, enabled = 1, disabled = 0.|
|SnapTempZone16MsmTemp|44 (0x2c)|F32|The temperature of temperature zone 16|
|SH_FswVersion|45 (0x2d)|U32|FSW version info|
|SnapdragonTotalCpuUtilTrunc|46 (0x2e)|U8|The total CPU utilization percentage, truncated to U8|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|SH_FileOpenError|0 (0x0)|Opening a file failed.| | | | |
| | | |type|I32||File type|
| | | |stat|I32||File error status|
|SH_FileReadError|1 (0x1)|Reading, seeking, etc of a file failed.| | | | |
| | | |type|I32||File type|
| | | |stat|I32||File error status|
|SH_FileWriteError|2 (0x2)|Writing a file failed.| | | | |
| | | |type|I32||File type|
| | | |stat|I32||File error status|
|SH_FswVersion|3 (0x3)|FSW version info| | | | |
| | | |hash|U32||FSW GIT hash|
| | | |branch|Fw::LogStringArg&|80|FSW GIT branch|
|SH_PowerSaver|4 (0x4)|Power saver state changed| | | | |
| | | |Mode|PowerSaverModeEvr|||

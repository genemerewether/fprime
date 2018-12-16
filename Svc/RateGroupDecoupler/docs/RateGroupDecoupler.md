<title>RateGroupDecoupler Component Dictionary</title>
# RateGroupDecoupler Component Dictionary


## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|RgDecoupMaxTime|0 (0x0)|U32|Max execution time rate group|
|RgDecoupCycleSlips|1 (0x1)|U32|Cycle slips for rate group|
|RgDecoupNumCycles|2 (0x2)|U32|Number of cycles|
|RgDecoupTime|3 (0x3)|U32|Execution time rate group|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|RgDecoupCycleSlip|0 (0x0)|Warning event that rate group has had a cycle slip| | | | |
| | | |cycle|U32||The cycle where the cycle occurred|    
|RgDecoupBackupCycle|1 (0x1)|Warning event that backup cycle is happening| | | | |
| | | |backupCycle|U32||The cycle where the cycle occurred|    
| | | |cycle|U32||The cycle where the cycle occurred|    
|RgDecoupRegularCycle|2 (0x2)|Event that regular cycle is happening again| | | | |
| | | |backupCycle|U32||The cycle where the cycle occurred|    
| | | |cycle|U32||The cycle where the cycle occurred|    

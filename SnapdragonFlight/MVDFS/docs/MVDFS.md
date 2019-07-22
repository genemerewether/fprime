<title>MVDFS Component Dictionary</title>
# MVDFS Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|MVDFS_Reinit|0 (0x0)|| | |
|MVDFS_Activate|1 (0x1)|| | |
| | | |enable|bool||

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|MVDFS_ImageCount|0 (0x0)|U32|MVDFS images processed|
|MVDFS_ImageRate|1 (0x1)|U32|MVDFS images processed per second|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|MVDFS_Activated|0 (0x0)|MVDFS began processing images| | | | |
|MVDFS_Deactivated|1 (0x1)|MVDFS stopped processing images| | | | |

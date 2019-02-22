<title>ImgComp Component Dictionary</title>
# ImgComp Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|IMGCOMP_NoOp|0 (0x0)|| | |   

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|IMGCOMP_BuffersHandled|0 (0x0)|U32|Number of buffers handled|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|IMGCOMP_SoftCompError|0 (0x0)|ImgComp had an error in software compression| | | | |
| | | |error|SoftCompErrorType||The error type|    
| | | |msg|Fw::LogStringArg&|40||    
|IMGCOMP_BadBuffer|1 (0x1)|Deserialization failed. This will cause leakage of a buffer| | | | |
|IMGCOMP_BadSetting|2 (0x2)|| | | | |
| | | |portNum|U32|||    
|IMGCOMP_NoBuffer|3 (0x3)|| | | | |
| | | |size|U32|||    
|IMGCOMP_BufferOffset|4 (0x4)|| | | | |
| | | |type|BufferOffsetSkipType||The error type|    
| | | |output|BufferOffsetSkipOutput||The error type|    
| | | |inBuffer|U32|||    
| | | |portNum|U32|||    
|IMGCOMP_NoRestartMarkers|5 (0x5)|ImgComp had an error using setenv| | | | |
| | | |error|I32|||    

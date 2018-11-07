<title>SerialTextConverter Component Dictionary</title>
# SerialTextConverter Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|STC_SET_MODE|1 (0x1)|Command to put in FILE (write texts to a file) or EVR (write texts to Evrs) mode.| | |   
| | | |mode|StcMode||                    


## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|STC_SerialText|0 (0x0)|Serial data received| | | | |
| | | |data|Fw::LogStringArg&|80|Serial data turned into a string|    
|STC_SerialReadBadStatus|1 (0x1)|Serial read had bad status.| | | | |
| | | |status|I32||Serial read status|    
|STC_SetMode_Cmd_Sent|2 (0x2)|STC_SET_MODE command sent| | | | |
| | | |mode|StcModeEv|||    
|STC_SetMode_Cmd_Invalid|3 (0x3)|STC_SET_MODE command sent to transition to FILE mode, but not log file has been specified.| | | | |
|STC_File|4 (0x4)|Displays the log file being used| | | | |
| | | |log_file|Fw::LogStringArg&|80|Log file being used|    

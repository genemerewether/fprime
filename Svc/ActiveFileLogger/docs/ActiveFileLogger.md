<title>ActiveFileLogger Component Dictionary</title>
# ActiveFileLogger Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|FLOG_RESET_LOG|0 (0x0)|Restart the file logs with a new time/date stamp| | |   
|FLOG_ACTIVATE|1 (0x1)|Activate logging| | |   
| | | |activate|bool|Activate/deactivate the logger|                    
|FLOG_RESET_LOG_DIR|2 (0x2)|Restart the file logs with a new time/date stamp in the specified base directory| | |   
| | | |dir_name|Fw::CmdStringArg|Base directory to start putting time stamped directories in for file logs.|                    


## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|AFL_ResetLogCmd|0 (0x0)|FLOG_RESET_LOG or FLOG_RESET_LOG_DIR command was sent| | | | |
| | | |dir|Fw::LogStringArg&|80|New time stamped directory for storing file logs.|    
|AFL_ActivateCmd|1 (0x1)|FLOG_ACTIVATE command was sent| | | | |
| | | |activate|bool||Activate/deactivate the logger|    

<title>FileDownlink Component Dictionary</title>
# FileDownlink Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|FILE_DWN_SEND_FILE|0 (0x0)|Read a named file off the disk. Divide it into packets and send the packets for transmission to the ground.| | |   
| | | |sourceFileName|Fw::CmdStringArg|The name of the on-board file to send|                    
| | | |destFileName|Fw::CmdStringArg|The name of the destination file on the ground|                    
|FILE_DWN_CANCEL|1 (0x1)|Cancel the downlink in progress, if any| | |   
|FILE_DWN_SEND_PARTIAL|2 (0x2)|Read a named file off the disk from a starting position. Divide it into packets and send the packets for transmission to the ground.| | |   
| | | |sourceFileName|Fw::CmdStringArg|The name of the on-board file to send|                    
| | | |destFileName|Fw::CmdStringArg|The name of the destination file on the ground|                    
| | | |startOffset|U32|Starting offset of the source file|                    
| | | |length|U32|Number of bytes to send from starting offset. Length of 0 implies until the end of the file|                    

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|FileDownlink_FilesSent|0 (0x0)|U32|The total number of files sent|
|FileDownlink_PacketsSent|1 (0x1)|U32|The total number of packets sent|
|FileDownlink_Warnings|2 (0x2)|U32|The total number of warnings|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|FileDownlink_FileOpenError|0 (0x0)|An error occurred opening a file| | | | |
| | | |fileName|Fw::LogStringArg&|100|The name of the file|    
|FileDownlink_FileReadError|1 (0x1)|An error occurred reading a file| | | | |
| | | |fileName|Fw::LogStringArg&|100|The name of the file|    
| | | |status|I32||The file status of read|    
|FileDownlink_FileSent|2 (0x2)|The File Downlink component successfully sent a file| | | | |
| | | |sourceFileName|Fw::LogStringArg&|100|The source file name|    
| | | |destFileName|Fw::LogStringArg&|100|The destination file name|    
|FileDownlink_DownlinkCanceled|3 (0x3)|The File Downlink component canceled downlink of a file| | | | |
| | | |sourceFileName|Fw::LogStringArg&|100|The source file name|    
| | | |destFileName|Fw::LogStringArg&|100|The destination file name|    
|FileDownlink_DownlinkTimeout|4 (0x4)|The File Downlink component has detected a timeout. Downlink has been canceled.| | | | |
| | | |sourceFileName|Fw::LogStringArg&|100|The source filename|    
| | | |destFileName|Fw::LogStringArg&|100|The destination file name|    
|FileDownlink_DownlinkPartialWarning|5 (0x5)|The File Downlink component has detected a timeout. Downlink has been canceled.| | | | |
| | | |startOffset|U32||Starting file offset in bytes|    
| | | |length|U32||Number of bytes to downlink|    
| | | |filesize|U32||Size of source file|    
| | | |sourceFileName|Fw::LogStringArg&|100|The source filename|    
| | | |destFileName|Fw::LogStringArg&|100|The destination file name|    
|FileDownlink_DownlinkPartialFail|6 (0x6)|The File Downlink component has detected a timeout. Downlink has been canceled.| | | | |
| | | |sourceFileName|Fw::LogStringArg&|100|The source filename|    
| | | |destFileName|Fw::LogStringArg&|100|The destination file name|    
| | | |startOffset|U32||Starting file offset in bytes|    
| | | |filesize|U32||Size of source file|    
|FileDownlink_SendDataFail|7 (0x7)|The File Downlink component generated an error when trying to send a data packet.| | | | |
| | | |sourceFileName|Fw::LogStringArg&|100|The source filename|    
| | | |byteOffset|U32||Byte offset|    
|FileDownlink_SendStarted|8 (0x8)|The File Downlink component started a file download.| | | | |
| | | |fileSize|U32||The source file size|    
| | | |sourceFileName|Fw::LogStringArg&|100|The source filename|    
| | | |destFileName|Fw::LogStringArg&|100|The destination filename|    

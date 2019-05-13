<title>BlspSerialDriver Component Dictionary</title>
# BlspSerialDriver Component Dictionary


## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|DR_BytesSent|0 (0x0)|U32|Bytes Sent|
|DR_BytesRecv|1 (0x1)|U32|Bytes Received|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|DR_OpenError|0 (0x0)|UART open error| | | | |
| | | |device|I32||The device|
| | | |error|I32||The error code|
|DR_ConfigError|1 (0x1)|UART config error| | | | |
| | | |device|I32||The device|
| | | |error|I32||The error code|
|DR_WriteError|2 (0x2)|UART write error| | | | |
| | | |device|I32||The device|
| | | |error|I32||The error code|
|DR_ReadError|3 (0x3)|UART read error| | | | |
| | | |device|I32||The device|
| | | |error|I32||The error code|
|DR_PortOpened|4 (0x4)|UART port opened event| | | | |
| | | |device|I32||The device|
|DR_NoBuffers|5 (0x5)|UART ran out of buffers| | | | |
| | | |device|I32||The device|
|DR_BufferTooSmall|6 (0x6)|UART ran out of buffers| | | | |
| | | |device|I32||The device|
| | | |size|U32||The provided buffer size|
| | | |needed|U32||The buffer size needed|

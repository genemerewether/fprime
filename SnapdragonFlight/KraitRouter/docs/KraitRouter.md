<title>KraitRouter Component Dictionary</title>
# KraitRouter Component Dictionary


## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|KR_ResyncErrors|0 (0x0)|U32|Number of packet resync errors|
|KR_NumPackets|1 (0x1)|U32|Number of packets from Krait|
|KR_NumDecodeErrors|2 (0x2)|U32|Number of port decode errors|
|KR_NumInvalidPorts|3 (0x3)|U32|Number of packets with invalid ports|
|KR_NumBadSerialPortCalls|4 (0x4)|U32|Number of bad Serial port calls, ie bad serialize status returned|
|KR_NumOutputBufferOverflows|5 (0x5)|U32|Number of output buffer overflows|
|KR_NumZeroPktSize|6 (0x6)|U32|Number of packets with zero size|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|KR_DataReceiveError|0 (0x0)|FastRPC or memory error| | | | |
| | | |error|KR_ReceiveError||The receive error code|    
| | | |stat|I32||Status code - depends on error type|    
|KR_InvalidPortNum|1 (0x1)|Input data packet had a invalid port number, ie port number is not connected.| | | | |
| | | |PortType|KR_PortType||Type of port that was not connected|    
| | | |portNum|U8|||    
|KR_BadSerialPortCall|2 (0x2)|Bad status returned from output Serial port call, ie HexPortsOut() call had issues deserializing the buffer.| | | | |
| | | |status|I32|||    
| | | |portNum|U8|||    
|KR_MemoryError|3 (0x3)|Allocate or deallocate error| | | | |
| | | |error|KR_MemoryError||Type of memory operation that failed|    
|KR_WriteError|4 (0x4)|Send error in calling FastRPC write function| | | | |
| | | |error|I32||The error code|    

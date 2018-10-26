<title>LLRouter Component Dictionary</title>
# LLRouter Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|LLR_SET_PATCH_MODE|0 (0x0)|No-op command| | |   
| | | |Mode|LLRPatchMode||                    
|LLR_LOAD_PATCH_FILE|1 (0x1)|Load and patch the LL| | |   
| | | |file|Fw::CmdStringArg|Patch file|                    
| | | |updatingLL|LL|The LL to update|                    
| | | |legacyBootloader|bool|Updating LLs with an old bootloader|                    
| | | |forceboot|bool|Tell the bootloader to ignore CRC32 checks on startup|                    
| | | |disabled_selftests|U64|Bitmask of tests to disable on startup. A 1 indicates that the test should not be run|                    
|LLR_CYCLE_SERIAL|2 (0x2)|Command to have scheduler port send serial data if GNC loop is not running| | |   
| | | |on|bool|Turn the command sending on or off.|                    

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|LLR_CrcErrors|0 (0x0)|U32|Number of CRC errors on receive|
|LLR_ResyncErrors|1 (0x1)|U32|Number of packet resync errors|
|LLR_NumPackets|2 (0x2)|U32|Number of packets from LL|
|LLR_NumSerialErrors|3 (0x3)|U32|Number of serial read errors|
|LLR_NumGoodPackets|4 (0x4)|U32|Number of good packets from LL, ie passed CRC check|
|LLR_NumDecodeErrors|5 (0x5)|U32|Number of packet decode errors on receive|
|LLR_NumInvalidPorts|6 (0x6)|U32|Number of packets with invalid ports|
|LLR_NumBadSerialPortCalls|7 (0x7)|U32|Number of bad Serial port calls, ie bad serialize status returned|
|LLR_NumOuputBufferOverflows|8 (0x8)|U32|Number of output buffer overflows|
|LLR_NumZeroPktSize|9 (0x9)|U32|Number of packets with zero size|
|LLR_MaxCycleTime|10 (0xa)|U32|Max GNC cycle time|
|LLR_CycleTime|11 (0xb)|U32|Current GNC cycle time|
|LLR_PatchProgress|12 (0xc)|U32|Number of patch bytes written|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|LLR_DataReceiveError|0 (0x0)|Serial receive error| | | | |
| | | |error|LLR_ReceiveError||The receive eror code|    
|LLR_DataCrcError|1 (0x1)|Serial CRC error| | | | |
| | | |expected|U16|||    
| | | |actual|U16|||    
|LLR_InvalidPortNum|2 (0x2)|Input serial data packet had a invalid port number, ie port number is not connected.| | | | |
| | | |portNum|U8|||    
|LLR_BadSerialPortCall|3 (0x3)|Bad status returned from output Serial port call, ie LLPortsOut() call had issues deserializing the buffer.| | | | |
| | | |status|I32|||    
| | | |portNum|U8|||    
|LLR_PatchFileOpen|4 (0x4)|LL Router successfully opened patch binary from filesystem.| | | | |
| | | |filename|Fw::LogStringArg&|40||    
| | | |filesize|U32|||    
|LLR_PatchModeChange|5 (0x5)|LL Router mode changed.| | | | |
| | | |newmode|Fw::LogStringArg&|20||    
|LLR_FirstACK|6 (0x6)|LL Router saw first ACK of the patch| | | | |
|LLR_PatchEvent|7 (0x7)|LL Router patch progress in Kilobytes.| | | | |
| | | |progress|U32|||    
|LLR_PatchComplete|8 (0x8)|LL Router patch completed.| | | | |
|LLR_PatchError|9 (0x9)|LL Router patch error.| | | | |
| | | |error|LLR_PatchError|||    
| | | |err_num|I32|||    
| | | |err_string|Fw::LogStringArg&|40||    
|LLR_PolyError|10 (0xa)|Error retrieving PolyDb value| | | | |

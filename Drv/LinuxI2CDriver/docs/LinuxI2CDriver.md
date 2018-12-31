<title>LinuxI2CDriver Component Dictionary</title>
# LinuxI2CDriver Component Dictionary


## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|I2C_ReadBytes|0 (0x0)|U32|Bytes Received|
|I2C_WriteBytes|1 (0x1)|U32|Bytes Sent|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|I2C_OpenError|0 (0x0)|I2C open error| | | | |
| | | |device|I32||The device|    
| | | |error|I32||The error code|    
|I2C_ConfigError|1 (0x1)|I2C config error| | | | |
| | | |device|I32||The device|    
| | | |error|I32||The error code|    
|I2C_WriteError|2 (0x2)|I2C write error| | | | |
| | | |device|I32||The device|    
| | | |error|I32||The error code|    
|I2C_PortOpened|4 (0x4)|I2C open notification| | | | |
| | | |device|I32||The device|    

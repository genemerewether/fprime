<title>BlspI2CDriver Component Dictionary</title>
# BlspI2CDriver Component Dictionary


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
|I2C_ReadWriteError|2 (0x2)|I2C write error| | | | |
| | | |device|I32||The device|    
| | | |addr|U32|||    
| | | |error|I32||The error code|    
| | | |bytesAct|I32|||    
| | | |bytesDes|I32|||    
|I2C_WriteError|3 (0x3)|I2C write error| | | | |
| | | |device|I32||The device|    
| | | |addr|U32|||    
| | | |error|I32||The error code|    
| | | |bytesAct|I32|||    
| | | |bytesDes|I32|||    
|I2C_PortOpened|4 (0x4)|I2C open notification| | | | |
| | | |device|I32||The device|    
|I2C_ReadError|5 (0x5)|I2C read error| | | | |
| | | |device|I32||The device|    
| | | |addr|U32|||    
| | | |error|I32||The error code|    
| | | |bytesAct|I32|||    
| | | |bytesDes|I32|||    
|I2C_BadReadWriteCall|6 (0x6)|I2C malformed request| | | | |
| | | |device|I32||The device|    

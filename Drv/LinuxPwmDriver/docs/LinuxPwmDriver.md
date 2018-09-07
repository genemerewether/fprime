<title>LinuxPwmDriver Component Dictionary</title>
# LinuxPwmDriver Component Dictionary



## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|PWM_OpenError|0 (0x0)|Open error| | | | |
| | | |chip|I32||The PWM chip|    
| | | |line|I32||The PWM line|    
| | | |error|I32||The error code|    
|PWM_ConfigError|1 (0x1)|PWM configure error| | | | |
| | | |chip|I32||The PWM chip|    
| | | |line|I32||The PWM line|    
| | | |error|I32||The error code|    
|PWM_WriteError|2 (0x2)|PWM write error| | | | |
| | | |chip|I32||The PWM chip|    
| | | |line|I32||The PWM line|    
| | | |error|I32||The error code|    
|PWM_ReadError|3 (0x3)|PWM read error| | | | |
| | | |chip|I32||The PWM chip|    
| | | |line|I32||The PWM line|    
| | | |error|I32||The error code|    
|PWM_PortOpened|4 (0x4)|PWM opened notification| | | | |
| | | |chip|I32||The PWM chip|    
| | | |line|I32||The PWM line|    

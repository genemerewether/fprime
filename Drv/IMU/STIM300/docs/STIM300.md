<title>STIM300 Component Dictionary</title>
# STIM300 Component Dictionary


## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|NumPackets|0 (0x0)|U32||
|ImuPacket|1 (0x1)|ROS::sensor_msgs::ImuNoCov||
|TimeSyncStatus|2 (0x2)|STIM300TimeSync||

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|BufferFull|0 (0x0)|| | | | |
|UartError|1 (0x1)|| | | | |
|NoEvents|2 (0x2)|| | | | |
|InvalidCounter|3 (0x3)|| | | | |
| | | |actualCount|U32|||
| | | |expectedCount|U32|||
|TooManyEvents|4 (0x4)|| | | | |
| | | |maxEvents|U32|||
|BadTimeSync|5 (0x5)|| | | | |
|SyncComplete|6 (0x6)|| | | | |

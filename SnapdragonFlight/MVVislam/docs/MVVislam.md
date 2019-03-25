<title>MVVislam Component Dictionary</title>
# MVVislam Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|MVVISLAM_Reinit|0 (0x0)|| | |   
|MVVISLAM_Activate|1 (0x1)|| | |   
| | | |enable|bool||                    
|MVVISLAM_Reset|2 (0x2)|| | |   

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|MVVISLAM_ErrorCode|0 (0x0)|U32|MVVislam error code|
|MVVISLAM_w_q_b|1 (0x1)|ROS::geometry_msgs::Quaternion|MVVislam orientation|
|MVVISLAM_x_b|2 (0x2)|ROS::geometry_msgs::Vector3|MVVislam translation|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|MVVISLAM_Reinitialized|0 (0x0)|MVVislam reinit| | | | |
|MVVISLAM_Reset|1 (0x1)|MVVislam reinit| | | | |

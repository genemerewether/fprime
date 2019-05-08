<title>StereoCam Component Dictionary</title>
# StereoCam Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|SCAM_SET_LOGGING|0 (0x0)|Turn on logging images to a file| | |   
| | | |Mode|LogMode||                    
|SCAM_ADD_WAYPOINT|1 (0x1)|Add a waypoint| | |   
| | | |waypoint|U32||                    
| | | |destination|WayptDestination||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|SCAM_TAKE_IMG|2 (0x2)|Take a still image| | |   
| | | |destination|TakeImgDestination||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|SCAM_ACTIVATE|3 (0x3)|Turn imaging on or off| | |   
| | | |Mode|StereoCamActive||                    
|SCAM_WAYPOINT_TEST|4 (0x4)|Manually trigger a waypoint image like port call would| | |   
| | | |index|U32||                    
|SCAM_TEST_IMG|5 (0x5)|Send a still image test pattern| | |   
| | | |destination|TestImgDestination|Manually keep in sync with CameraFrameSerializable|                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|SCAM_FLIGHT_MODE|6 (0x6)|Toggle delivery of images to GNC and frame-to-frame autoexposure clamp| | |   
| | | |mode|StereoCamFlightMode||                    
|SCAM_IMG_TLM_MODE|7 (0x7)|Toggle image telemetry| | |   
| | | |mode|StereoCamImgTlmMode||                    
|SCAM_SET_LOG_SKIP|8 (0x8)|| | |   
| | | |skipCount|U32||                    

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|SCAM_ImagesAcquired|0 (0x0)|U32|Number of images acquired|
|SCAM_Exposure|1 (0x1)|U32|Exposure time setting|
|SCAM_Gain|2 (0x2)|U32|Gain setting|
|SCAM_ExposureCalc|3 (0x3)|F32|Calculated exposure time setting (before int cast)|
|SCAM_GainCalc|4 (0x4)|F32|Calculated gain setting (before int cast)|
|SCAM_ImageRate|5 (0x5)|U32|Number of frames per second|
|SCAM_ImageTimeoutLeft|6 (0x6)|U32|Number of seconds remaining on image timeout|
|SCAM_NumNoHPBuffers|7 (0x7)|U32||
|SCAM_NumNoLPBuffers|8 (0x8)|U32||
|SCAM_NumLoRate|9 (0x9)|U32||
|SCAM_NumGncBufferReturnQueueErrs|10 (0xa)|U32||
|SCAM_NumMaxExpDelta|11 (0xb)|U32||

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|SCAM_UninitializedError|0 (0x0)|StereoCam received a command while the component was uninitialized| | | | |
|SCAM_AlreadyInitializedError|1 (0x1)|StereoCam asked to initialize while already initialized| | | | |
|SCAM_DeactivatedError|2 (0x2)|StereoCam tried to wait for an image callback while deactivated| | | | |
|SCAM_ExtraPreviewCallbackError|3 (0x3)|StereoCam received an image callback while (de)activating| | | | |
|SCAM_CameraError|4 (0x4)|StereoCam error with Snapdragon Linux Camera API| | | | |
| | | |error|SCAM_CameraErrorType||The error code|    
|SCAM_TookImg|5 (0x5)|StereoCam took an image using the SCAM_TAKE_IMG command| | | | |
|SCAM_Activated|6 (0x6)|StereoCam began dispatching images| | | | |
|SCAM_Deactivated|7 (0x7)|StereoCam stopped dispatching images| | | | |
|SCAM_LoggingModeChange|8 (0x8)|StereoCam logging mode changed| | | | |
| | | |Mode|LoggingModeEVR|||    
|SCAM_NoHPBuffers|9 (0x9)|StereoCam ran out of buffers on the specified High Priority port| | | | |
| | | |port|U32|||    
|SCAM_NoLPBuffers|10 (0xa)|StereoCam ran out of buffers on the specified Low Priority port| | | | |
| | | |port|U32|||    
|SCAM_LoRate|11 (0xb)|StereoCam captured less than SCAM_MIN_RATE_WARN images since last Sched call| | | | |
| | | |numImages|U32|||    
|SCAM_ExposureModeChange|12 (0xc)|StereoCam switched from fixed to autoexposure or vice versa| | | | |
| | | |Mode|ExposureModeEVR|||    
|SCAM_BusyWaitingImage|13 (0xd)|StereoCam busy waiting for previous image to be acquired| | | | |
| | | |waypoint|I32||-1 for the waypoint value means StereoCam was asked for a manual image|    
|SCAM_MissedWaypoint|14 (0xe)|StereoCam did not take an image at waypoint because it was deactivated| | | | |
| | | |waypoint|U32|||    
|SCAM_WaypointError|15 (0xf)|StereoCam waypoint had spec error| | | | |
| | | |waypoint|U32|||    
| | | |type|U32|||    
|SCAM_NoWaypointsLeft|16 (0x10)|StereoCam used all the waypoint slots| | | | |
|SCAM_WaypointAdded|17 (0x11)|StereoCam added a waypoint| | | | |
| | | |waypoint|U32|||    
| | | |type|U32|||    
| | | |quality|U8|||    
|SCAM_NoWaypointSpec|18 (0x12)|StereoCam did not take an image at waypoint - not found in list| | | | |
| | | |waypoint|U32|||    
|SCAM_WaypointRecv|19 (0x13)|StereoCam received a waypoint notification| | | | |
| | | |waypoint|U32|||    
|SCAM_MaxExposureDelta|20 (0x14)|StereoCam unable to change exposure because max delta was hit| | | | |
| | | |oldExp|U32|||    
| | | |newExp|U32|||    
|SCAM_FlightModeChange|21 (0x15)|StereoCam switched from preflight to flight or vice versa| | | | |
| | | |Mode|FlightModeEVR|||    
|SCAM_ImgTlmModeChange|22 (0x16)|StereoCam turned image telemetry on or off| | | | |
| | | |Mode|ImgTlmModeEVR|||    
|SCAM_OverrodeQuality|23 (0x17)|| | | | |
| | | |oldQual|U32|||    
| | | |newQual|U32|||    
|SCAM_CallbackTypeChange|24 (0x18)|StereoCam switched between 8-bit and 10-bit callbacks| | | | |
| | | |Type|CallbackTypeEVR|||    
|SCAM_GncBufferReturnQueueErr|25 (0x19)|| | | | |
| | | |errStat|I32|||    

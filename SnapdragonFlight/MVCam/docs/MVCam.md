<title>MVCam Component Dictionary</title>
# MVCam Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|MVCAM_SET_LOGGING|0 (0x0)|Turn on logging images to a file| | |   
| | | |Mode|LogMode||                    
|MVCAM_ADD_WAYPOINT|1 (0x1)|Add a waypoint| | |   
| | | |waypoint|U32||                    
| | | |destination|WayptDestination||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|MVCAM_TAKE_IMG|2 (0x2)|Take a still image| | |   
| | | |destination|TakeImgDestination||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|MVCAM_ACTIVATE|3 (0x3)|Turn imaging on or off| | |   
| | | |Mode|MVCamActive||                    
|MVCAM_WAYPOINT_TEST|4 (0x4)|Manually trigger a waypoint image like port call would| | |   
| | | |index|U32||                    
|MVCAM_TEST_IMG|5 (0x5)|Send a still image test pattern| | |   
| | | |destination|TestImgDestination|Manually keep in sync with CameraFrameSerializable|                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|MVCAM_FLIGHT_MODE|6 (0x6)|Toggle delivery of images to GNC and frame-to-frame autoexposure clamp| | |   
| | | |mode|MVCamFlightMode||                    
|MVCAM_IMG_TLM_MODE|7 (0x7)|Toggle image telemetry| | |   
| | | |mode|MVCamImgTlmMode||                    
|MVCAM_SET_LOG_SKIP|8 (0x8)|| | |   
| | | |skipCount|U32||                    

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|MVCAM_ImagesAcquired|0 (0x0)|U32|Number of images acquired|
|MVCAM_Exposure|1 (0x1)|U32|Exposure time setting|
|MVCAM_Gain|2 (0x2)|U32|Gain setting|
|MVCAM_ExposureCalc|3 (0x3)|F32|Calculated exposure time setting (before int cast)|
|MVCAM_GainCalc|4 (0x4)|F32|Calculated gain setting (before int cast)|
|MVCAM_ImageRate|5 (0x5)|U32|Number of frames per second|
|MVCAM_ImageTimeoutLeft|6 (0x6)|U32|Number of seconds remaining on image timeout|
|MVCAM_NumNoHPBuffers|7 (0x7)|U32||
|MVCAM_NumNoLPBuffers|8 (0x8)|U32||
|MVCAM_NumLoRate|9 (0x9)|U32||
|MVCAM_NumGncBufferReturnQueueErrs|10 (0xa)|U32||
|MVCAM_NumMaxExpDelta|11 (0xb)|U32||

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|MVCAM_UninitializedError|0 (0x0)|MVCam received a command while the component was uninitialized| | | | |
|MVCAM_AlreadyInitializedError|1 (0x1)|MVCam asked to initialize while already initialized| | | | |
|MVCAM_DeactivatedError|2 (0x2)|MVCam tried to wait for an image callback while deactivated| | | | |
|MVCAM_ExtraPreviewCallbackError|3 (0x3)|MVCam received an image callback while (de)activating| | | | |
|MVCAM_CameraError|4 (0x4)|MVCam error with Snapdragon Linux Camera API| | | | |
| | | |error|MVCAM_CameraErrorType||The error code|    
|MVCAM_TookImg|5 (0x5)|MVCam took an image using the MVCAM_TAKE_IMG command| | | | |
|MVCAM_Activated|6 (0x6)|MVCam began dispatching images| | | | |
|MVCAM_Deactivated|7 (0x7)|MVCam stopped dispatching images| | | | |
|MVCAM_LoggingModeChange|8 (0x8)|MVCam logging mode changed| | | | |
| | | |Mode|LoggingModeEVR|||    
|MVCAM_NoHPBuffers|9 (0x9)|MVCam ran out of buffers on the specified High Priority port| | | | |
| | | |port|U32|||    
|MVCAM_NoLPBuffers|10 (0xa)|MVCam ran out of buffers on the specified Low Priority port| | | | |
| | | |port|U32|||    
|MVCAM_LoRate|11 (0xb)|MVCam captured less than MVCAM_MIN_RATE_WARN images since last Sched call| | | | |
| | | |numImages|U32|||    
|MVCAM_ExposureModeChange|12 (0xc)|MVCam switched from fixed to autoexposure or vice versa| | | | |
| | | |Mode|ExposureModeEVR|||    
|MVCAM_BusyWaitingImage|13 (0xd)|MVCam busy waiting for previous image to be acquired| | | | |
| | | |waypoint|I32||-1 for the waypoint value means MVCam was asked for a manual image|    
|MVCAM_MissedWaypoint|14 (0xe)|MVCam did not take an image at waypoint because it was deactivated| | | | |
| | | |waypoint|U32|||    
|MVCAM_WaypointError|15 (0xf)|MVCam waypoint had spec error| | | | |
| | | |waypoint|U32|||    
| | | |type|U32|||    
|MVCAM_NoWaypointsLeft|16 (0x10)|MVCam used all the waypoint slots| | | | |
|MVCAM_WaypointAdded|17 (0x11)|MVCam added a waypoint| | | | |
| | | |waypoint|U32|||    
| | | |type|U32|||    
| | | |quality|U8|||    
|MVCAM_NoWaypointSpec|18 (0x12)|MVCam did not take an image at waypoint - not found in list| | | | |
| | | |waypoint|U32|||    
|MVCAM_WaypointRecv|19 (0x13)|MVCam received a waypoint notification| | | | |
| | | |waypoint|U32|||    
|MVCAM_MaxExposureDelta|20 (0x14)|MVCam unable to change exposure because max delta was hit| | | | |
| | | |oldExp|U32|||    
| | | |newExp|U32|||    
|MVCAM_FlightModeChange|21 (0x15)|MVCam switched from preflight to flight or vice versa| | | | |
| | | |Mode|FlightModeEVR|||    
|MVCAM_ImgTlmModeChange|22 (0x16)|MVCam turned image telemetry on or off| | | | |
| | | |Mode|ImgTlmModeEVR|||    
|MVCAM_OverrodeQuality|23 (0x17)|| | | | |
| | | |oldQual|U32|||    
| | | |newQual|U32|||    
|MVCAM_CallbackTypeChange|24 (0x18)|MVCam switched between 8-bit and 10-bit callbacks| | | | |
| | | |Type|CallbackTypeEVR|||    
|MVCAM_GncBufferReturnQueueErr|25 (0x19)|| | | | |
| | | |errStat|I32|||    

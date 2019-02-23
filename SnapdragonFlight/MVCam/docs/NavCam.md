<title>NavCam Component Dictionary</title>
# NavCam Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|NCAM_SET_LOGGING|0 (0x0)|Turn on logging images to a file| | |   
| | | |Mode|LogMode||                    
|NCAM_ADD_WAYPOINT|1 (0x1)|Add a waypoint| | |   
| | | |waypoint|U32||                    
| | | |destination|WayptDestination||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|NCAM_TAKE_IMG|2 (0x2)|Take a still image| | |   
| | | |destination|TakeImgDestination||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|NCAM_ACTIVATE|3 (0x3)|Turn imaging on or off| | |   
| | | |Mode|NavCamActive||                    
|NCAM_TOGGLE_FIXED_EXPOSURE|4 (0x4)|Switch between fixed- and auto-exposure| | |   
| | | |Mode|ExposureMode||                    
|NCAM_SET_EXPOSURE_GAIN|5 (0x5)|Set fixed exposure and gain values| | |   
| | | |exposure|U32||                    
| | | |gain|U32||                    
|NCAM_OVERRIDE_POSTPROC|6 (0x6)|Set camera pipeline postproc parameters| | |   
| | | |brightness|U32|min=0, max=6, step=1|                    
| | | |contrast|U32|min=0, max=10, step=1|                    
| | | |iso|PostProcISO||                    
| | | |sharpness|U32|min=0, max=36, step=6|                    
|NCAM_WAYPOINT_TEST|7 (0x7)|Manually trigger a waypoint image like port call would| | |   
| | | |index|U32||                    
|NCAM_TEST_IMG|8 (0x8)|Send a still image test pattern| | |   
| | | |destination|TestImgDestination|Manually keep in sync with CameraFrameSerializable|                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|NCAM_SET_EXP_PARMS|9 (0x9)|Set the NavCam MV CPA exposure parameters; defaults in Cfg.hpp| | |   
| | | |max_change|U16|Max frame to frame exposure change; between 1 and 500, inclusive|                    
| | | |exposure_cost|F32|MV CPA cost mode exposure cost|                    
| | | |gain_cost|F32|MV CPA cost mode gain cost|                    
| | | |max_exposure|U16|MV CPA max exposure|                    
| | | |max_gain|U16|MV CPA max gain|                    
|NCAM_FLIGHT_MODE|10 (0xa)|Toggle frame notifications to VisionNav and images to VisionProc| | |   
| | | |mode|NcamFlightMode||                    
|NCAM_IMG_TLM_MODE|11 (0xb)|Toggle image telemetry| | |   
| | | |mode|NcamImgTlmMode||                    
|NCAM_SET_CALLBACK|12 (0xc)|Switch between 8-bit and 10-bit callbacks| | |   
| | | |Mode|CallbackType||                    
|NCAM_SET_LOG_SKIP|13 (0xd)|| | |   
| | | |skipCount|U32||                    

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|NCAM_ImagesAcquired|0 (0x0)|U32|Number of images acquired|
|NCAM_Exposure|1 (0x1)|U32|Exposure time setting|
|NCAM_Gain|2 (0x2)|U32|Gain setting|
|NCAM_ExposureCalc|3 (0x3)|F32|Calculated exposure time setting (before int cast)|
|NCAM_GainCalc|4 (0x4)|F32|Calculated gain setting (before int cast)|
|NCAM_ImageRate|5 (0x5)|U32|Number of frames per second|
|NCAM_ImageTimeoutLeft|6 (0x6)|U32|Number of seconds remaining on image timeout|
|NCAM_NumNoHPBuffers|7 (0x7)|U32||
|NCAM_NumNoLPBuffers|8 (0x8)|U32||
|NCAM_NumLoRate|9 (0x9)|U32||
|NCAM_NumGncBufferReturnQueueErrs|10 (0xa)|U32||
|NCAM_NumMaxExpDelta|11 (0xb)|U32||

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|NCAM_UninitializedError|0 (0x0)|NavCam received a command while the component was uninitialized| | | | |
|NCAM_AlreadyInitializedError|1 (0x1)|NavCam asked to initialize while already initialized| | | | |
|NCAM_DeactivatedError|2 (0x2)|NavCam tried to wait for an image callback while deactivated| | | | |
|NCAM_ExtraPreviewCallbackError|3 (0x3)|NavCam received an image callback while (de)activating| | | | |
|NCAM_CameraError|4 (0x4)|NavCam error with Snapdragon Linux Camera API| | | | |
| | | |error|NCAM_CameraErrorType||The error code|    
|NCAM_TookImg|5 (0x5)|NavCam took an image using the NCAM_TAKE_IMG command| | | | |
|NCAM_Activated|6 (0x6)|NavCam began dispatching images| | | | |
|NCAM_Deactivated|7 (0x7)|NavCam stopped dispatching images| | | | |
|NCAM_LoggingModeChange|8 (0x8)|NavCam logging mode changed| | | | |
| | | |Mode|LoggingModeEVR|||    
|NCAM_NoHPBuffers|9 (0x9)|NavCam ran out of buffers on the specified High Priority port| | | | |
| | | |port|U32|||    
|NCAM_NoLPBuffers|10 (0xa)|NavCam ran out of buffers on the specified Low Priority port| | | | |
| | | |port|U32|||    
|NCAM_LoRate|11 (0xb)|NavCam captured less than NCAM_MIN_RATE_WARN images since last Sched call| | | | |
| | | |numImages|U32|||    
|NCAM_ExposureModeChange|12 (0xc)|NavCam switched from fixed to autoexposure or vice versa| | | | |
| | | |Mode|ExposureModeEVR|||    
|NCAM_BusyWaitingImage|13 (0xd)|NavCam busy waiting for previous image to be acquired| | | | |
| | | |waypoint|I32||-1 for the waypoint value means NavCam was asked for a manual image|    
|NCAM_MissedWaypoint|14 (0xe)|NavCam did not take an image at waypoint because it was deactivated| | | | |
| | | |waypoint|U32|||    
|NCAM_WaypointError|15 (0xf)|NavCam waypoint had spec error| | | | |
| | | |waypoint|U32|||    
| | | |type|U32|||    
|NCAM_NoWaypointsLeft|16 (0x10)|NavCam used all the waypoint slots| | | | |
|NCAM_WaypointAdded|17 (0x11)|NavCam added a waypoint| | | | |
| | | |waypoint|U32|||    
| | | |type|U32|||    
| | | |quality|U8|||    
|NCAM_NoWaypointSpec|18 (0x12)|NavCam did not take an image at waypoint - not found in list| | | | |
| | | |waypoint|U32|||    
|NCAM_WaypointRecv|19 (0x13)|NavCam received a waypoint notification| | | | |
| | | |waypoint|U32|||    
|NCAM_MaxExposureDelta|20 (0x14)|NavCam unable to change exposure because max delta was hit| | | | |
| | | |oldExp|U32|||    
| | | |newExp|U32|||    
|NCAM_FlightModeChange|21 (0x15)|NavCam switched from preflight to flight or vice versa| | | | |
| | | |Mode|FlightModeEVR|||    
|NCAM_ImgTlmModeChange|22 (0x16)|NavCam turned image telemetry on or off| | | | |
| | | |Mode|ImgTlmModeEVR|||    
|NCAM_OverrodeQuality|23 (0x17)|| | | | |
| | | |oldQual|U32|||    
| | | |newQual|U32|||    
|NCAM_CallbackTypeChange|24 (0x18)|NavCam switched between 8-bit and 10-bit callbacks| | | | |
| | | |Type|CallbackTypeEVR|||    
|NCAM_GncBufferReturnQueueErr|25 (0x19)|| | | | |
| | | |errStat|I32|||    

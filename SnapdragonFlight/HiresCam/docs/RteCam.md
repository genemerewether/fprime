<title>RteCam Component Dictionary</title>
# RteCam Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|RCAM_SET_LOGGING|0 (0x0)|Turn on logging images to a file| | |   
| | | |Mode|LogMode||                    
|RCAM_ADD_WAYPOINT|1 (0x1)|Add a waypoint| | |   
| | | |waypoint|U32||                    
| | | |destination|WayptDestination||                    
| | | |action|WayptAction||                    
| | | |type|WayptCaptureType||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|RCAM_TAKE_IMG|2 (0x2)|Take a still image| | |   
| | | |destination|TakeImgDestination||                    
| | | |type|ImgType||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|RCAM_TAKE_VIDEO|3 (0x3)|Take a video| | |   
| | | |setting|VidType||                    
| | | |frames|U32||                    
|RCAM_WAYPOINT_TEST|4 (0x4)|Manually trigger a waypoint image like port call would| | |   
| | | |index|U32||                    
|RCAM_SET_VID_TIMEOUTS|5 (0x5)|Set frame count and elapsed time timeouts for video| | |   
| | | |frames|U32||                    
| | | |sched|U32||                    
|RCAM_TEST_IMG|6 (0x6)|Send a still image test pattern| | |   
| | | |destination|TestImgDestination|Manually keep in sync with CameraFrameSerializable|                    
| | | |type|TestImgType||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|RCAM_SET_VID_SKIP|7 (0x7)|Set frame skip count for video, e.g. 2 delivers every 3rd frame| | |   
| | | |skip|U32||                    
|RCAM_ACTIVATE|8 (0x8)|Turn imaging on or off| | |   
| | | |Mode|RteCamActive||                    

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|RCAM_ImagesAcquired|0 (0x0)|U32|Number of images acquired|
|RCAM_FramesAcquired|1 (0x1)|U32|Number of video frames acquired total|
|RCAM_FrameRate|2 (0x2)|U32|Number of frames per second|
|RCAM_FramesAcquiredThisTime|3 (0x3)|U32|Number of video frames acquired for current capture command|
|RCAM_TimeoutLeft|4 (0x4)|U32|Number of seconds remaining on image / video timeout|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|RCAM_UninitializedError|0 (0x0)|RteCam received a command while the component was unitialized| | | | |
|RCAM_AlreadyInitializedError|1 (0x1)|RteCam asked to initialize while already initialized| | | | |
|RCAM_DeactivatedError|2 (0x2)|RteCam tried to wait for an image callback while deactivated| | | | |
|RCAM_ExtraCallbackError|3 (0x3)|RteCam received a callback while (de)activating| | | | |
| | | |error|RCAM_ExtraCallbackType||The error code|    
|RCAM_CameraError|4 (0x4)|RteCam error with Snapdragon Linux Camera API| | | | |
| | | |error|RCAM_CameraErrorType||The error code|    
|RCAM_TookImg|5 (0x5)|RteCam took an image using the RCAM_TAKE_IMG command| | | | |
|RCAM_Activated|6 (0x6)|RteCam began dispatching images| | | | |
|RCAM_Deactivated|7 (0x7)|RteCam stopped dispatching images| | | | |
|RCAM_NoBuffers|8 (0x8)|RteCam ran out of buffers on the specified port| | | | |
| | | |port|U32|||    
|RCAM_LoggingModeChange|9 (0x9)|RteCam logging mode changed| | | | |
| | | |Mode|LoggingModeEVR|||    
|RCAM_LoRate|10 (0xa)|RteCam captured less than RCAM_MIN_RATE_WARN images since last Sched call| | | | |
| | | |numImages|U32|||    
|RCAM_BusyWaiting|11 (0xb)|RteCam busy waiting for previous operation to complete| | | | |
| | | |waypoint|I32||-1 for the waypoint value means RteCam was asked for a manual action|    
|RCAM_WaypointError|12 (0xc)|RteCam waypoint had spec error| | | | |
| | | |waypoint|U32|||    
| | | |dest|U32|||    
| | | |action|U32|||    
| | | |stillType|U32|||    
| | | |vidType|U32|||    
|RCAM_NoWaypointsLeft|13 (0xd)|RteCam used all the waypoint slots| | | | |
|RCAM_WaypointAdded|14 (0xe)|RteCam added a waypoint| | | | |
| | | |waypoint|U32|||    
| | | |dest|U32|||    
| | | |action|U32|||    
| | | |type|U32|||    
| | | |quality|U8|||    
|RCAM_NoWaypointSpec|15 (0xf)|RteCam did nothing at waypoint - not found in list| | | | |
| | | |waypoint|U32|||    
|RCAM_WaypointRecv|16 (0x10)|RteCam received a waypoint notification| | | | |
| | | |waypoint|U32|||    
|RCAM_OverrodeQuality|17 (0x11)|| | | | |
| | | |oldQual|U32|||    
| | | |newQual|U32|||    

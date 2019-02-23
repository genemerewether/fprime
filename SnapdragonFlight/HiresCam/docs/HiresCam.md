<title>HiresCam Component Dictionary</title>
# HiresCam Component Dictionary


## Command List

|Mnemonic|ID|Description|Arg Name|Arg Type|Comment
|---|---|---|---|---|---|
|HIRESCAM_SET_LOGGING|0 (0x0)|Turn on logging images to a file| | |   
| | | |Mode|LogMode||                    
|HIRESCAM_ADD_WAYPOINT|1 (0x1)|Add a waypoint| | |   
| | | |waypoint|U32||                    
| | | |destination|WayptDestination||                    
| | | |action|WayptAction||                    
| | | |type|WayptCaptureType||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|HIRESCAM_TAKE_IMG|2 (0x2)|Take a still image| | |   
| | | |destination|TakeImgDestination||                    
| | | |type|ImgType||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|HIRESCAM_TAKE_VIDEO|3 (0x3)|Take a video| | |   
| | | |setting|VidType||                    
| | | |frames|U32||                    
|HIRESCAM_WAYPOINT_TEST|4 (0x4)|Manually trigger a waypoint image like port call would| | |   
| | | |index|U32||                    
|HIRESCAM_SET_VID_TIMEOUTS|5 (0x5)|Set frame count and elapsed time timeouts for video| | |   
| | | |frames|U32||                    
| | | |sched|U32||                    
|HIRESCAM_TEST_IMG|6 (0x6)|Send a still image test pattern| | |   
| | | |destination|TestImgDestination|Manually keep in sync with CameraFrameSerializable|                    
| | | |type|TestImgType||                    
| | | |quality|U8|Compression quality; between 1 and 100, inclusive; greater than 100 is passthrough|                    
|HIRESCAM_SET_VID_SKIP|7 (0x7)|Set frame skip count for video, e.g. 2 delivers every 3rd frame| | |   
| | | |skip|U32||                    
|HIRESCAM_ACTIVATE|8 (0x8)|Turn imaging on or off| | |   
| | | |Mode|HiresCamActive||                    

## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|HIRESCAM_ImagesAcquired|0 (0x0)|U32|Number of images acquired|
|HIRESCAM_FramesAcquired|1 (0x1)|U32|Number of video frames acquired total|
|HIRESCAM_FrameRate|2 (0x2)|U32|Number of frames per second|
|HIRESCAM_FramesAcquiredThisTime|3 (0x3)|U32|Number of video frames acquired for current capture command|
|HIRESCAM_TimeoutLeft|4 (0x4)|U32|Number of seconds remaining on image / video timeout|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|HIRESCAM_UninitializedError|0 (0x0)|HiresCam received a command while the component was unitialized| | | | |
|HIRESCAM_AlreadyInitializedError|1 (0x1)|HiresCam asked to initialize while already initialized| | | | |
|HIRESCAM_DeactivatedError|2 (0x2)|HiresCam tried to wait for an image callback while deactivated| | | | |
|HIRESCAM_ExtraCallbackError|3 (0x3)|HiresCam received a callback while (de)activating| | | | |
| | | |error|HIRESCAM_ExtraCallbackType||The error code|    
|HIRESCAM_CameraError|4 (0x4)|HiresCam error with Snapdragon Linux Camera API| | | | |
| | | |error|HIRESCAM_CameraErrorType||The error code|    
|HIRESCAM_TookImg|5 (0x5)|HiresCam took an image using the HIRESCAM_TAKE_IMG command| | | | |
|HIRESCAM_Activated|6 (0x6)|HiresCam began dispatching images| | | | |
|HIRESCAM_Deactivated|7 (0x7)|HiresCam stopped dispatching images| | | | |
|HIRESCAM_NoBuffers|8 (0x8)|HiresCam ran out of buffers on the specified port| | | | |
| | | |port|U32|||    
|HIRESCAM_LoggingModeChange|9 (0x9)|HiresCam logging mode changed| | | | |
| | | |Mode|LoggingModeEVR|||    
|HIRESCAM_LoRate|10 (0xa)|HiresCam captured less than HIRESCAM_MIN_RATE_WARN images since last Sched call| | | | |
| | | |numImages|U32|||    
|HIRESCAM_BusyWaiting|11 (0xb)|HiresCam busy waiting for previous operation to complete| | | | |
| | | |waypoint|I32||-1 for the waypoint value means HiresCam was asked for a manual action|    
|HIRESCAM_WaypointError|12 (0xc)|HiresCam waypoint had spec error| | | | |
| | | |waypoint|U32|||    
| | | |dest|U32|||    
| | | |action|U32|||    
| | | |stillType|U32|||    
| | | |vidType|U32|||    
|HIRESCAM_NoWaypointsLeft|13 (0xd)|HiresCam used all the waypoint slots| | | | |
|HIRESCAM_WaypointAdded|14 (0xe)|HiresCam added a waypoint| | | | |
| | | |waypoint|U32|||    
| | | |dest|U32|||    
| | | |action|U32|||    
| | | |type|U32|||    
| | | |quality|U8|||    
|HIRESCAM_NoWaypointSpec|15 (0xf)|HiresCam did nothing at waypoint - not found in list| | | | |
| | | |waypoint|U32|||    
|HIRESCAM_WaypointRecv|16 (0x10)|HiresCam received a waypoint notification| | | | |
| | | |waypoint|U32|||    
|HIRESCAM_OverrodeQuality|17 (0x11)|| | | | |
| | | |oldQual|U32|||    
| | | |newQual|U32|||    

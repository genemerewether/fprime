<title>ZmqSub Component Dictionary</title>
# ZmqSub Component Dictionary


## Telemetry Channel List

|Channel Name|ID|Type|Description|
|---|---|---|---|
|ZS_PacketsReceived|0 (0x0)|U32|Number of packets received|

## Event List

|Event Name|ID|Description|Arg Name|Arg Type|Arg Size|Description
|---|---|---|---|---|---|---|
|ZS_SubscribeConnectionOpened|0 (0x0)|Connection opened| | | | |
|ZS_ContextError|1 (0x1)|Zmq subscriber context open error| | | | |
| | | |error|Fw::LogStringArg&|80||
|ZS_SocketError|2 (0x2)|Zmq subscriber socket error| | | | |
| | | |error|Fw::LogStringArg&|80||
|ZS_ConnectError|3 (0x3)|Zmq subscriber connect error| | | | |
| | | |error|Fw::LogStringArg&|80||
|ZS_ReceiveError|4 (0x4)|Zmq subscriber receive error| | | | |
| | | |error|Fw::LogStringArg&|80||
|ZS_SockOptsError|5 (0x5)|Zmq subscriber socket options error| | | | |
| | | |error|Fw::LogStringArg&|80||

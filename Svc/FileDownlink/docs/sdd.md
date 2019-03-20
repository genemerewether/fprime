<title>Svc::FileDownlink</title>

# Svc::FileDownlink

## 1 Introduction

`FileDownlink` is an active ISF component.
It manages downlink of files from the spacecraft.

## 2 Requirements

Requirement | Description | Rationale | Verification Method
---- | ---- | ---- | ----
ISF-FD-001 | Upon command, `FileDownlink` shall read a file from non-volatile storage, partition the file into packets, and send out the packets. | This requirement provides the capability to downlink files from the spacecraft. | Test


## 3 Design

### 3.1 Assumptions

The design of `FileDownlink` assumes the following:

1. File downlink occurs by dividing files into packets
of type [`Fw::FilePacket`](../../../Fw/FilePacket/docs/sdd.html).

2. One file downlink happens at a time.

### 3.2 Block Description Diagram (BDD)

![`FileDownlink` BDD](img/FileDownlinkBDD.jpg "FileDownlink")

### 3.3 Ports

#### 3.3.1 Role Ports

Name | Type | Role
-----| ---- | ----
`timeCaller` | [`Fw::Time`](../../../Fw/Time/docs/sdd.html) | TimeGet
`cmdIn` | [`Fw::Cmd`](../../../Fw/Cmd/docs/sdd.html) | Cmd
`cmdRegOut` | [`Fw::CmdReg`](../../../Fw/Cmd/docs/sdd.html) | CmdReg
`cmdResponseOut` | [`Fw::CmdResponse`](../../../Fw/Cmd/docs/sdd.html) | CmdResponse
`tlmOut` | [`Fw::Tlm`](../../../Fw/Tlm/docs/sdd.html) | Telemetry
`eventOut` | [`Fw::LogEvent`](../../../Fw/Log/docs/sdd.html) | LogEvent
`Run` | [`Svc::Sched`](../../../Svc/Sched/docs/sdd.html) | Sched

#### 3.3.2 Component-Specific Ports

Name | Type | Kind | Purpose
---- | ---- | ---- | ----
<a name="bufferGet">`bufferGetCaller`</a> | [`Fw::BufferGet`](../../../Fw/Buffer/docs/sdd.html) | output (caller) | Requests buffers for sending file packets.
<a name="bufferSendOut">`bufferSendOut`</a> | [`Fw::BufferSend`](../../../Fw/Buffer/docs/sdd.html) | output | Sends buffers containing file packets.
<a name="bufferSendOut">`bufferReturn`</a> | [`Fw::BufferSend`](../../../Fw/Buffer/docs/sdd.html) | input | Reuse buffer provided and continue downlinking data.
<a name="bufferSendOut">`buffSendOutReturn`</a> | [`Fw::BufferSend`](../../../Fw/Buffer/docs/sdd.html) | output | Returns buffer to manager when downlink has completed. 

### 3.4 Constants

`FileDownlink` has the following constants, initialized
at component instantiation time:

* *downlinkPacketSize*:
The size of the packets to use on downlink.
* *timeout*: Timeout threshold (milliseconds) while in the WAIT state.
* *cycleTime*: Rate at which the component is running. 

### 3.5 State

`FileDownlink` maintains a *mode* equal to
one of the following values:

* IDLE (0): `FileDownlink` is idle.

* DOWNLINK (1): `FileDownlink` is performing a file downlink.

* CANCEL (2): `FileDownlink` is canceling a file downlink.

* WAIT (3): `FileDownlink` is waiting for a buffer to continue file downlink.

The initial value is IDLE.

### 3.6 Commands

`FileDownlink` recognizes the commands described in the following sections.

#### 3.6.1 Send File

Send File is an asynchronous command.
It has two arguments:

1. *sourceFileName*:
The name of the on-board file to send.

2. *destFileName*:
The name of the destination file on the ground.

When `FileDownlink` receives this command, it carries
out the following steps:

1. If *mode* = CANCEL, then 

    a. Issue a *DownlinkCanceled* event.

    b. set *mode* = IDLE and return.

2. Set *mode* = DOWNLINK.

3. Open the file *sourceFileName* for reading with file descriptor *d*.
If there is any problem opening the file, then issue a
*FileOpenError* warning and abort the command execution.

4. Invoke *bufferGetCaller*
to request a buffer whose size is *downlinkPacketSize*.
Fill the buffer with the START packet and send it out on *bufferSendOut*.

5. Set *mode = WAIT*.

6. When the buffer has been returned via the *bufferReturn* port, set *mode = DOWNLINK*, and
send the first DATA packet.

7. Set *mode = WAIT*.

8. When the buffer has been returned via the *bufferReturn* port, set *mode = DOWNLINK*, and
continue sending DATA packets.

9. If there is any problem reading the file, then issue a
*FileReadError* warning event, close the file, and abort the command execution.

10. Repeat steps 7 and 8 until we have sent every DATA packet. Once completed and we have the buffer,
send the END packet and issue a *FileSent* event.

7. Close the file.

10. Set *mode = IDLE*.

#### 3.6.2 Send Partial File

Send Partial File is an asynchronous command. It has four arguments:

1. *sourceFileName*:
The name of the on-board file to send.

2. *destFileName*:
The name of the destination file on the ground.

3. *startOffset*:
The starting offset of the source file.

4. *length*:
The number of bytes to downlink from the start offset.

The method of sending data packets is identical to the *Send File* command with the exception of the 
starting offset and length. In the *Send File* command, the *startOffet* is 0 and *length* is the 
file size.


#### 3.6.3 Cancel

Cancel is a synchronous command.
If *mode* = DOWNLINK, it sets *mode* to CANCEL.
Otherwise it does nothing.

## 4 Dictionary

[Link](FileDownlink.html)

## 5 Checklists

Document | Link
-------- | ----
Design | [Link](Checklist/design.xlsx)
Code | [Link](Checklist/code.xlsx)
Unit Test | [Link](Checklist/unit_test.xls)

## 6 Unit Testing

TODO

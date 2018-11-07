<title>LLProc::LLDebug Component SDD</title>
# LLProc::LLDebug Component

## 1. Introduction

The `LLProc::LLDebug` component provides a way to send text debug messages from components in the LLProc to the HLProc. It also captures FW_ASSERT calls and sends the text to the HLProc before setting the fault discrete.

## 2. Requirements

The requirements for `LLProc::LLDebug` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------
LL-DBG-001 | The `LLProc::LLDebug` component shall forward debug text to the connected serial port | Unit Test
LL-DBG-002 | The `LLProc::LLDebug` component shall intercept FW_ASSERT calls | Unit Test
LL-DBG-003 | The `LLProc::LLDebug` component shall send the assert message to the connected serial port | Unit Test
LL-DBG-004 | The `LLProc::LLDebug` component shall assert the fault GPIO after sending the fault text | Unit Test

## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

The `LLProc::LLDebug` component has the following component diagram:

![`LLProc::LLDebug` Diagram](img/LLDebugBDD.jpg "LLProc::LLDebug")

#### 3.1.2 Ports

The `LLProc::LLDebug` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage |
-------------- | ---- | --------- | ---- | ----- |
|`Drv::SerialWrite`[HTML](../../../Drv/SerialDriverPorts/docs/sdd.html)[MD](../../../Drv/SerialDriverPorts/docs/sdd.md) | SerWritePort | output | n/a | Port to send serial text|
|`R5::GpioSet`[HTML](../../../R5/Ports/docs/sdd.html)[MD](../../../R5/Ports/docs/sdd.md) | SwFault | output | n/a | Port to set fault discrete|
|`LLProc::DebugString`[HTML](../../../LLProc/Ports/docs/sdd.html)[MD](../../../LLProc/Ports/docs/sdd.md) | DebugStringIn | input | n/a | Port to receive debug text|


### 3.2 Functional Description

The `LLProc::LLDebug` component is a passive component that receives debug text and FW_ASSERT invocations.

#### 3.2.1 Debug Text

It is sometimes desirable to send debugging text while developing modules on the LLProc. `STDOUT` is not available, so another text path is needed. Modules can connect to the `DebugStringIn` port and send messages. The `LLProc::LLDebug` component copies this text to a `Fw::Buffer` instance and sends it out the driver port.

#### 3.2.2 FW_ASSERT execution

The debug component intercepts any calls to `FW_ASSERT` by registering itself as an assert hook in the F' architecture. `LLProc::LLDebug` is derived from `Fw::AssertHook`, so it uses base class method `registerHook()` to register. Then it implements the defined `reportAssert()`, `printAssert()`, and `doAssert()`. The `doAssert()` method invokes the `SwFault` port to set the fault GPIO. The `doAssert()` method then calls C `assert()`. When a debugger is connected, the halted LLProc can be examined for the cause of the assert. 

### 3.3 Scenarios

TODO

### 3.4 State

`LLProc::LLDebug` has no state machines.

### 3.5 Algorithms

`LLProc::LLDebug` has no significant algorithms.

## 4. Dictionaries

There are no dictionaries for this component.

## 5. Module Checklists

N/A

## 6. Unit Testing
The unit tests check the two major functions of the component.

## 6.1 Unit Test Descriptions
### 6.1.1 Debug Test

Tests sending debug text and verifying it was sent to serial port.

Requirement verified: LL-DBG-001

### 6.1.2 Assert Text

Tests sending an assert and verifying text sent and GPIO set.

Requirement verified:  LL-DBG-002, LL-DBG-003, LL-DBG-004

## 6.2 Unit Test Output
[Unit Test Output](../test/ut/output/test.txt)

## 6.3 Unit Test Coverage
[Coverage Summary](../test/ut/output/LLDebug_gcov.txt)

[Implementation Coverage](../test/ut/output/LLDebugComponentImpl.cpp.gcov)

[Autocoded Coverage](../test/ut/output/LLDebugComponentAc.cpp.gcov)

## 7. Change Log

Date | Description
---- | -----------
1/4/2018 | Initial SDD


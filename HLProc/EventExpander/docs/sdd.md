<title>Svc::EventExpander Component SDD</title>
# Svc::EventExpander Component

## 1. Introduction

UART ports have limited bandwidth, so sending full-sized events from the low-level to the high-level processor can quickly overwhelm the remaining bandwidth. This component works in conjunction with the `LLProc/ShortLogQueue` component to buffer and send short versions of events. The short version of the event has the time tag stripped and the severit reduced to one byte. The `Svc::EventExpander` component has an input port to take the shortened events from the LLProc processor and convert them back to full events. It uses system time on the HLProc processor to time tag the events.

## 2. Requirements

The requirements for `Svc::EventExpander` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------
SVC-EE-001 | The `Svc::EventExpander` component shall convert short event invocations and convert them to full F' events | Unit Test
SVC-EE-002 | The `Svc::EventExpander` component shall timestamp the short event with the system time | Unit Test

## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

The `Svc::EventExpander` component has the following component diagram:

![`Svc::EventExpander` Diagram](img/EventExpander.jpg "Svc::EventExpander")

#### 3.1.2 Ports

The `Svc::EventExpander` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage
-------------- | ---- | --------- | ---- | -----
[`Common::ShortLog`](../../../Common/Ports/docs/sdd.html) | LogRecv | input | sync_input | Input port with short event
[`Fw::Log`](../../../Fw/Log/docs/sdd.html) | LogSend | output | n/a | Port to send buffers for the serial driver to use
[`Fw::Time`](../../../Fw/Time/docs/sdd.html) | Time | output | n/a | Port to send buffers for the serial driver to use


### 3.2 Functional Description



### 3.3 Scenarios

`Svc::EventExpander` has no significant scenarios.

### 3.4 State

`Svc::EventExpander` has no state machines.

### 3.5 Algorithms

`Svc::EventExpander` has no significant algorithms.

## 4. Dictionaries

`Svc::EventExpander` has no dictionaries.

## 5. Module Checklists

TODO

## 6. Unit Testing
The unit test verifies that a short event is successfully converted into a regular event.

## 6.1 Unit Test Descriptions
### 6.1.1 Nominal

Nominal testing for short to normal event conversion.

Requirement verified: `SVC-EE-001`, `SVC-EE-002`

## 6.2 Unit Test Output
[Unit Test Output](../test/ut/output/test.txt)

## 6.3 Unit Test Coverage
[EventExpander coverage](../test/ut/output/EventExpanderComponentImpl.cpp.gcov)

[Coverage Output](../test/ut/output/SVCEventExpander_gcov.txt)


## 7. Change Log

Date | Description
---- | -----------
8/20/2017 | Initial SDD




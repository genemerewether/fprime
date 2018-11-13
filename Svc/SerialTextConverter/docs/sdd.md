<title>Svc::SerialTextConverter Component SDD</title>
# Svc::SerialTextConverter Component

## 1. Introduction

The `Svc::SerialTextConverter` component is responsible for receiving serial data from the debug UART, and converting the data to Events, ie Events with a single string argument.

## 2. Requirements

The requirements for `Svc::SerialTextConverter` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------
SVC-STC-001 | The `Svc::SerialTextConverter` component shall provide input port for reading serial data. | Unit Test
SVC-STC-002 | The `Svc::SerialTextConverter` component shall provide a output port to return `Fw::Buffer` to the serial driver. | Unit Test
SVC-STC-003 | The `Svc::SerialTextConverter` component shall emit DIAGNOSTIC Events with a single string argument for each serial buffer read. | Unit Test
SVC-STC-004 | The `Svc::SerialTextConverter` component shall split up serial data into strings by the null terminator character up to max size of [40]. | Unit Test

## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

The `Svc::SerialTextConverter` component has the following component diagram:

![`Svc::SerialTextConverter` Diagram](img/SerialTextConverterBDD.jpg "Svc::SerialTextConverter")

#### 3.1.2 Ports

The `Svc::SerialTextConverter` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage
-------------- | ---- | --------- | ---- | -----
[`Drv::SerialRead`](../../../Drv/SerialDriverPorts/docs/sdd.html) | SerReadPort | input | sync_input | Input port for serial driver to send serial data to read
[`Fw::BufferSend`](../../../Fw/Buffer/docs/sdd.html) | SerialBufferSend | output | n/a | Port to send buffers for the serial driver to use


### 3.2 Functional Description



### 3.3 Scenarios

`Svc::SerialTextConverter` has no significant scenarios.

### 3.4 State

`Svc::SerialTextConverter` has no state machines.

### 3.5 Algorithms

`Svc::SerialTextConverter` has no significant algorithms.

## 4. Dictionaries

[Dictionaries](SerialTextConverter.html)

## 5. Module Checklists

TODO

## 6. Unit Testing
The SerialTextConverter unit tests are designed to test interfaces and functionality with the available ports, command processing, telemetry output, EVR and data product generation.

## 6.1 Unit Test Descriptions
### 6.1.1 Test All

Nominal/Off-nominal testing for the log text conversion.

Requirement verified: `SVC-STC-001`, `SVC-STC-002`, `SVC-STC-003`, `SVC-STC-004`

## 6.2 Unit Test Output
[Unit Test Output](../test/ut/ut_output.txt)

## 6.3 Unit Test Coverage
[Coverage Output](../test/ut/SerialTextConverter_gcov.txt)


## 7. Change Log

Date | Description
---- | -----------
5/9/2017 | Initial SDD




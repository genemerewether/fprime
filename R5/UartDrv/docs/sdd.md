<title>R5::UartDriver Component SDD</title>
# R5::UartDriver Component

## 1. Introduction

The `R5::UartDriver` is responsible for communicating over the UART.
Multiple data transfers can be performed (transmit and receive) in single R5 cycle.
Transmit data transfers are completely independent of receive data transfers and asynchronous from them.
Received data can be retrieved through the receive port.

## 2. Requirements

The requirements for `R5::UartDriver` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------

## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

The `R5::UartDriver` component has the following component diagram:

![`R5::UartDriver` Diagram](img/UartDriverBDD.jpg "UartDriver")

#### 3.1.2 Ports

The `R5::UartDriver` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage
-------------- | ---- | --------- | ---- | -----
[`Drv::SerialWrite`] | m_serialSend_InputPort | Output | n/a | Send data
[`Drv::SerialRead`] | m_serialRecv_InputPort | Input | n/a | Receive data

### 3.2 Functional Description

The `R5::UartDriver` component provides communication through the UART transmit and receive interfaces.
<br>Both transmit and receive interfaces implemented by the `R5::UartDriver` component configure and use DMA channels to insure the most efficient use of available R5 hardware resources (fully concurrent operations of UART transmit and receive channels and CPU).
<br>The receive DMA channel is configured in such a way that after filling a buffer it continues writing new data at the beginning of the buffer, effectively implementing a ring buffer functionality.
<br>After data transfer is initiated the R5 CPU is free to perform other operations, while the UART interfaces and DMA channels facilitate data transmit and receive.
<br>
<br>There are 2 buffers (transmit buffer and receive buffer) are allocated by the `R5::UartDriver` component during initialization.
To insure data coherency, while both of those buffers accessed by the R5 CPU and R5 DMA, the buffers must be allocated from a special R5 memory region configured with proper caching policy.

### 3.3 Scenarios

If transmit interface is activated before the previous transmit transaction has completed, the CPU will poll the DMA status register (in busy loop) until the previous transaction completes and then proceed with setting up next transaction.
<br>When receive interface is activated, the R5 DMA controller facilities are accessed to get the information of the location of the last data written to the receive buffer.
The data received since the last activation of the receive interface (expected to happen each cycle, but could be a complete receive buffer in case of cycle slip) is copied from the receive buffer into the buffer passed to the receive interface.
<br>The buffer passed to a receive interface must be large enough to accommodate all the data from the `R5::UartDriver` component receive buffer.

### 3.4 State

`R5::UartDriver` has no state machines.

### 3.5 Algorithms

`R5::UartDriver` has no significant algorithms.

## 4. Dictionary

Dictionaries: [HTML](UartDriver.html) [MD](UartDriver.md)

## 4. Module Checklists

Document | Link
-------- | ----
Design Checklist | [Link](Checklist_Design.xlsx)
Code Checklist | [Link](Checklist_Code.xlsx)
Unit Test Checklist | [Link](Checklist_Unit_Test.xls)

## 5. Unit Testing

The unit test must be compiled for and executed on the R5 hardware (TMS570LC4357 LaunchPad).

[Unit Test Output](../test/ut/output/test.txt)

[Coverage Summary](../test/ut/output/SvcUartDriver_gcov.txt)

[Coverage Output - `CommandDispatcherImpl.cpp`](../test/ut/output/CommandDispatcherImpl.cpp.gcov)

[Coverage Output - `CommandDispatcherComponentAc.cpp`](../test/ut/output/CommandDispatcherComponentAc.cpp.gcov)

## 6. Change Log

Date | Description
---- | -----------
 |




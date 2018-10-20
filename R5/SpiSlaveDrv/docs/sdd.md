<title>R5::SpiSlaveDriver Component SDD</title>
# R5::SpiSlaveDriver Component

## 1. Introduction

The `R5::SpiSlaveDriver` is responsible for receiving data through the SPI Slave interface.
The received packets are accumulated in the `R5::SpiSlaveDriver` buffer and can be retrieved through the receive port.

## 2. Requirements

The requirements for `R5::SpiSlaveDriver` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------

## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

The `R5::SpiSlaveDriver` component has the following component diagram:

![`R5::SpiSlaveDriver` Diagram](img/SpiSlaveDrvBDD.jpg "SpiSlaveDriver")

#### 3.1.2 Ports

The `R5::SpiSlaveDriver` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage
-------------- | ---- | --------- | ---- | -----
[`R5::InputSpiReceivePort`] | m_spiRecv_InputPort | Input | n/a | Receive data

### 3.2 Functional Description

The `R5::SpiSlaveDriver` component provides reception of data through the SPI Slave interface.
<br>The receive interface implemented by the `R5::SpiSlaveDriver` component configures and uses a DMA channel to insure the most efficient use of available R5 hardware resources (fully concurrent operations of SPI Slave channel and CPU).
<br>The DMA channel is configured in such a way that after filling a buffer it continues writing new data at the beginning of the buffer, effectively implementing a ring buffer functionality.
<br>After data transfer is initiated all data transfers to the R5 memory happen asynchronously.
<br>The R5 CPU performs other operations, while SPI interface and DMA channels facilitate the data receive.
<br>
<br>The receive buffer is allocated by the `R5::SpiSlaveDriver` component during initialization.
To insure data coherency, while the receive buffer accessed by the R5 CPU and R5 DMA, the buffer must be allocated from a special R5 memory region configured with proper caching policy.

### 3.3 Scenarios

When receive interface is activated, the R5 DMA controller facilities are accessed to get the information of the location of the last data written to the receive buffer.
The data received since the last activation of the receive interface (expected to happen each cycle, but could be a complete receive buffer in case of cycle slip) is copied from the receive buffer into the buffer passed to the receive interface.
<br>The buffer passed to the receive interface must be large enough to accommodate all the data from the `R5::SpiSlaveDriver` component receive buffer.

### 3.4 State

`R5::SpiSlaveDriver` has no state machines.

### 3.5 Algorithms

`R5::SpiSlaveDriver` has no significant algorithms.

## 4. Dictionary

Dictionaries: [HTML](SpiSlaveDriver.html) [MD](SpiSlaveDriver.md)

## 4. Module Checklists

Document | Link
-------- | ----
Design Checklist | [Link](Checklist_Design.xlsx)
Code Checklist | [Link](Checklist_Code.xlsx)
Unit Test Checklist | [Link](Checklist_Unit_Test.xls)

## 5. Unit Testing

The unit test must be compiled for and executed on the R5 hardware (TMS570LC4357 LaunchPad).

[Unit Test Output](../test/ut/output/test.txt)

[Coverage Summary](../test/ut/output/SvcSpiSlaveDriver_gcov.txt)

[Coverage Output - `CommandDispatcherImpl.cpp`](../test/ut/output/CommandDispatcherImpl.cpp.gcov)

[Coverage Output - `CommandDispatcherComponentAc.cpp`](../test/ut/output/CommandDispatcherComponentAc.cpp.gcov)

## 6. Change Log

Date | Description
---- | -----------
 |




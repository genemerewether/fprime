<title>R5::SpiMasterDriver Component SDD</title>
# R5::SpiMasterDriver Component

## 1. Introduction

The `R5::SpiMasterDriver` is responsible for communicating through the SPI Master interface.
Multiple packets can be sent in single transaction.
During each send transaction equal number of bytes are received into the driver's receive buffer.
Received data can be retrieved through the receive port.

## 2. Requirements

The requirements for `R5::SpiMasterDriver` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------

## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

The `R5::SpiMasterDriver` component has the following component diagram:

![`R5::SpiMasterDriver` Diagram](img/SpiMasterDriverBDD.jpg "SpiMasterDriver")

#### 3.1.2 Ports

The `R5::SpiMasterDriver` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage
-------------- | ---- | --------- | ---- | -----
[`R5::InputSpiSendPort`] | m_spiSend_InputPort | Output | n/a | Send packets
[`R5::InputSpiReceivePort`] | m_spiRecv_InputPort | Input | n/a | Receive data

### 3.2 Functional Description

The `R5::SpiMasterDriver` component provides communication through the SPI Master interface.
<br>Both send and receive interfaces implemented by the `R5::SpiMasterDriver` component configure and use DMA channels to insure the most efficient use of available R5 hardware resources (fully concurrent operations of SPI Master channel and CPU).
<br>After data transfer is initiated the R5 CPU is free to perform other operations, while SPI interface and DMA channels facilitate the data transmit and receive.
<br>
<br>There are 2 buffers (transmit buffer and receive buffer) are allocated by the `R5::SpiMasterDriver` component during initialization.
To insure data coherency, while both of those buffers accessed by the R5 CPU and R5 DMA, the buffers must be allocated from a special R5 memory region configured with proper caching policy.

### 3.3 Scenarios

If send interface is activated before the previous transaction has completed, the CPU will poll the DMA status register (in busy loop) until the previous transaction completes and then proceed with setting up next transaction.
<br>The buffer passed to a receive interface must be large enough to accommodate all the data from the `R5::SpiMasterDriver` component receive buffer.

### 3.4 State

`R5::SpiMasterDriver` has no state machines.

### 3.5 Algorithms

`R5::SpiMasterDriver` has no significant algorithms.

## 4. Dictionary

Dictionaries: [HTML](SpiMasterDriver.html) [MD](SpiMasterDriver.md)

## 4. Module Checklists

Document | Link
-------- | ----
Design Checklist | [Link](Checklist_Design.xlsx)
Code Checklist | [Link](Checklist_Code.xlsx)
Unit Test Checklist | [Link](Checklist_Unit_Test.xls)

## 5. Unit Testing

The unit test must be compiled for and executed on the R5 hardware (TMS570LC4357 LaunchPad).

[Unit Test Output](../test/ut/output/test.txt)

[Coverage Summary](../test/ut/output/SvcSpiMasterDriver_gcov.txt)

[Coverage Output - `CommandDispatcherImpl.cpp`](../test/ut/output/CommandDispatcherImpl.cpp.gcov)

[Coverage Output - `CommandDispatcherComponentAc.cpp`](../test/ut/output/CommandDispatcherComponentAc.cpp.gcov)

## 6. Change Log

Date | Description
---- | -----------
 |




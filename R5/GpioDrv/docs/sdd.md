<title>R5::GpioDriver Component SDD</title>
# R5::GpioDriver Component

## 1. Introduction

The `R5::GpioDriver` is responsible for reading and setting the state of the bits of two R5 GIO ports (A and B).

## 2. Requirements

The requirements for `R5::GpioDriver` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------

## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

The `R5::GpioDriver` component has the following component diagram:

![`R5::GpioDriver` Diagram](img/GpioDrvBDD.jpg "GpioDriver")

#### 3.1.2 Ports

The `R5::GpioDriver` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage
-------------- | ---- | --------- | ---- | -----
[`R5::InputGpioSetPort`] | m_set_InputPort | Output | n/a | Set the state of the bit of the GIO port
[`R5::InputGpioSetPort`] | m_get_InputPort | Input | n/a | Get the state of the bit of the GIO port
[`R5::InputGpioSetPort`] | m_wait_InputPort | Input | n/a | Wait for the state change of the bit of the GIO port

### 3.2 Functional Description

The `R5::GpioDriver` provides an interface to obtain the state of bit the R5 GIO ports.
<br>The `R5::GpioDriver` provides an interface to set the state of the bit R5 GIO ports ("0"/Low or "1"/High).
<br>The `R5::GpioDriver` provides an interface wait for the bit of the R5 GIO ports to change its state from Low to High.

### 3.3 Scenarios

### 3.4 State

`R5::GpioDriver` has no state machines.

### 3.5 Algorithms

`R5::GpioDriver` has no significant algorithms.

## 4. Dictionary

Dictionaries: [HTML](GpioDriver.html) [MD](GpioDriver.md)

## 4. Module Checklists

Document | Link
-------- | ----
Design Checklist | [Link](Checklist_Design.xlsx)
Code Checklist | [Link](Checklist_Code.xlsx)
Unit Test Checklist | [Link](Checklist_Unit_Test.xls)

## 5. Unit Testing

The unit test must be compiled for and executed on the R5 hardware (TMS570LC4357 LaunchPad).

[Unit Test Output](../test/ut/output/test.txt)

[Coverage Summary](../test/ut/output/SvcGpioDriver_gcov.txt)

[Coverage Output - `CommandDispatcherImpl.cpp`](../test/ut/output/CommandDispatcherImpl.cpp.gcov)

[Coverage Output - `CommandDispatcherComponentAc.cpp`](../test/ut/output/CommandDispatcherComponentAc.cpp.gcov)

## 6. Change Log

Date | Description
---- | -----------
 |




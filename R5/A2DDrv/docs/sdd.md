<title>R5::A2DDriver Component SDD</title>
# R5::A2DDriver Component

## 1. Introduction

The `R5::A2DDriver` is responsible for reading the value of the Analog-to-Digital converter channel.

## 2. Requirements

The requirements for `R5::A2DDriver` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------

## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

#### 3.1.2 Ports

The `R5::A2DDriver` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage
-------------- | ---- | --------- | ---- | -----
[`R5::R5::InputA2DGetPort`] | m_get_InputPort | Input | n/a | Get the value of the Analog-to-Digital converter channel

### 3.2 Functional Description

The `R5::A2DDriver` provides an interface to get the value of the Analog-to-Digital converter channel.
<br>The retrieved "raw" value is converted to 32 Bit floating point "voltage" representation before it's returned to a caller.

### 3.3 Scenarios

### 3.4 State

`R5::A2DDriver` has no state machines.

### 3.5 Algorithms

`R5::A2DDriver` has no significant algorithms.

## 4. Dictionary

Dictionaries: [HTML](A2DDriver.html) [MD](A2DDriver.md)

## 4. Module Checklists

Document | Link
-------- | ----
Design Checklist | [Link](Checklist_Design.xlsx)
Code Checklist | [Link](Checklist_Code.xlsx)
Unit Test Checklist | [Link](Checklist_Unit_Test.xls)

## 5. Unit Testing

The unit test must be compiled for and executed on the R5 hardware (TMS570LC4357 LaunchPad).

[Unit Test Output](../test/ut/output/test.txt)

[Coverage Summary](../test/ut/output/SvcA2DDriver_gcov.txt)

[Coverage Output - `CommandDispatcherImpl.cpp`](../test/ut/output/CommandDispatcherImpl.cpp.gcov)

[Coverage Output - `CommandDispatcherComponentAc.cpp`](../test/ut/output/CommandDispatcherComponentAc.cpp.gcov)

## 6. Change Log

Date | Description
---- | -----------
 |




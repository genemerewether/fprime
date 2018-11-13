<title>R5::DmaAllocator Component SDD</title>
# R5::DmaAllocator Component

## 1. Introduction

The `R5::DmaAllocator` is responsible for allocating (with HalCoGen) a DMA safe memory region and managing the DMA safe memory.
DMA safe memory can be only allocated from the `R5::DmaAllocator`.
The deallocate() interface provided for interface completeness only.
Once memory is allocated from the `R5::DmaAllocator` can not be dynamically returned for reuse.

## 2. Requirements

The requirements for `R5::DmaAllocator` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------

## 3. Design

### 3.1 Context

### 3.2 Functional Description

The `R5::DmaAllocator` provides interface to obtain a DMA safe memory/buffer.
The buffer is allocated on the cache line boundary (32 bytes) and it's size is rounded up to the cache line size (32 bytes).

### 3.3 Scenarios

If allocation request succeeds, the address of the DMA safe memory is returned.
If allocation request fails, "0" is returned.

### 3.4 State

`R5::DmaAllocator` has no state machines.

### 3.5 Algorithms

`R5::DmaAllocator` has no significant algorithms.

## 4. Dictionary

Dictionaries: [HTML](DmaAllocator.html) [MD](DmaAllocator.md)

## 4. Module Checklists

Document | Link
-------- | ----
Design Checklist | [Link](Checklist_Design.xlsx)
Code Checklist | [Link](Checklist_Code.xlsx)
Unit Test Checklist | [Link](Checklist_Unit_Test.xls)

## 5. Unit Testing

The unit test must be compiled for and executed on the R5 hardware (TMS570LC4357 LaunchPad).

[Unit Test Output](../test/ut/output/test.txt)

[Coverage Summary](../test/ut/output/SvcDmaAllocator_gcov.txt)

[Coverage Output - `CommandDispatcherImpl.cpp`](../test/ut/output/CommandDispatcherImpl.cpp.gcov)

[Coverage Output - `CommandDispatcherComponentAc.cpp`](../test/ut/output/CommandDispatcherComponentAc.cpp.gcov)

## 6. Change Log

Date | Description
---- | -----------
 |




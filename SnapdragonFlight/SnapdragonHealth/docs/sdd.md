<title>SnapdragonFlight::SnapdragonHealth</title>

# SnapdragonFlight::SnapdragonHealth

## 1 Introduction

`SnapdragonHealth` is an active component that periodically pushes temperature zone info, and CPU Utilization to telemetry.

## 2 Requirements

Requirement | Description | Rationale | Verification Method
---- | ---- | ---- | ----
LEO-HEALTH-001 | The `SnapdragonHealth` component shall telemetery temperature zone every second. | | Unit testing
LEO-HEALTH-002 | The `SnapdragonHealth` component shall telemetery CPU utilization for every core every second. | | Unit testing

## 3 Design

### 3.1 Assumptions



### 3.2 Block Description Diagram (BDD)

![`SnapdragonHealth` BDD](img/SnapdragonHealthBDD.jpg "SnapdragonHealth")

### 3.3 Ports

#### 3.3.1 Role Ports

Name | Type | Role
-----| ---- | ----
`Time` | [`Fw::Time`](../../../Fw/Tlm/docs/sdd.html)  | TimeGet
`Tlm` | [`Fw::Tlm`](../../../Fw/Tlm/docs/sdd.html) | Telemetry
`Log` | [`Fw::Log`](../../../Fw/Log/docs/sdd.html) | LogEvent
`LogText` | [`Fw::LogText`](../../../Fw/Log/docs/sdd.html) | LogText

#### 3.3.2 Component-Specific Ports

Name | Type | Kind | Purpose
---- | ---- | ---- | -------
`SchedIn`| [`Svc::Sched`](../../../Svc/Sched/docs/sdd.html) | async_input | The rate group scheduler input port, which is called at 1 Hz.            


### 3.4 Functional Description


### 3.5 Constants

`SnapdragonHealth` does not have any constants.

### 3.6 State

`SnapdragonHealth` does not have significant state.

### 3.7 Commands

`SnapdragonHealth` does not have any commands.

## 4 Dictionary

[Link](SnapdragonHealth.html)

## 5 Checklists

Document | Link
-------- | ----
Design | [Link](Checklist/design.xlsx)
Code | [Link](Checklist/code.xlsx)
Unit Test | [Link](Checklist/unit_test.xls)

## 6 Unit Testing

[Test Output](../test/ut/output/test_out.txt)

[Coverage Report](../test/ut/output/SnapdragonFlightSnapdragonHealth_gcov.txt)


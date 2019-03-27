<title>SnapdragonFlight::HiresCam Component SDD</title>
# SnapdragonFlight::HiresCam Component

## 1. Introduction

The `SnapdragonFlight::HiresCam` component is responsible for acquiring images or videos from the forward facing Hires camera. The images from this camera are saved to storage for downlink. The component will be loaded with a set of waypoints and associated camera setting (image vs. video, resolution, etc).

## 2. Requirements

The requirements for `SnapdragonFlight::HiresCam` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------
NAV-RCAM-001 | The `SnapdragonFlight::HiresCam` component shall accept [TBD] scheduled waypoints with camera settings | Unit Test
NAV-RCAM-002 | The `SnapdragonFlight::HiresCam` component shall take the specified image when the waypoint arrival is sent | Unit Test
NAV-RCAM-003 | The `SnapdragonFlight::HiresCam` component shall save the image to a file along with helicopter pose data | Unit Test
NAV-RCAM-004 | The `SnapdragonFlight::HiresCam` component shall use one waypoint to start video and another to stop it. The video will will be saved after the end of the video | Unit Test
NAV-RCAM-005 | The `SnapdragonFlight::HiresCam` component shall stop video recording after [TBD] cycles to prevent filling storage with a missed end waypoint | Unit Test
NAV-RCAM-006 | The `SnapdragonFlight::HiresCam` component shall have commands for taking single images and videos | Unit Test

## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

#### 3.1.2 Ports

The `SnapdragonFlight::HiresCam` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage
-------------- | ---- | --------- | ---- | -----
[`Gnc::Waypoint`](../../../Gnc/GncComp/GncPorts/WaypointPortAi.xml) | WaypointRecv | input | async_input | Port for Gnc to trigger image capture via a waypoint

### 3.2 Functional Description

#### 3.2.1 Image Capture

The `SnapdragonFlight::HiresCam` component uses the Qualcomm libcamera API to capture images from the
IMX214 optic flow camera on the Sunny AW13V04 module.

### 3.3 Scenarios

`SnapdragonFlight::HiresCam` has no significant scenarios.

### 3.4 State

`SnapdragonFlight::HiresCam` has no state machines.

### 3.5 Algorithms

`SnapdragonFlight::HiresCam` has no significant algorithms.

## 4. Dictionaries

[Dictionaries](RteCam.html)

## 5. Module Checklists

N/A

## 6. Unit Testing
The RteCam unit tests are designed to test interfaces and functionality with the available ports, command processing, telemetry output, EVR and data product generation.

## 6.1 Unit Test Descriptions

## 6.2 Unit Test Output

## 6.3 Unit Test Coverage


## 7. Change Log

Date | Description
---- | -----------

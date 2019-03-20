<title>SnapdragonFlight::MVCam Component SDD</title>
# SnapdragonFlight::MVCam Component

## 1. Introduction

The `SnapdragonFlight::MVCam` component is responsible for acquiring and forwarding images from the downward facing 640x480 optic flow camera. The images from this camera are sent to the Visual Navigation component for processing, and to image compression / logging pathways.

## 2. Requirements

The requirements for `SnapdragonFlight::MVCam` are as follows:

Requirement | Description | Verification Method
----------- | ----------- | -------------------
NAV-NCAM-001 | The `SnapdragonFlight::MVCam` component shall acquire [30] images per second, driven by the OV7251 and Sunny MD102-A camera electronics | Unit Test
NAV-NCAM-002 | The `SnapdragonFlight::MVCam` component shall log images when requested | Unit Test
NAV-NCAM-004 | The `SnapdragonFlight::MVCam` component shall forward the image to the ImageSend port when images are cycling | Unit Test
NAV-NCAM-005 | The `SnapdragonFlight::MVCam` component shall log images based on receipt of a waypoint | Unit Test
NAV-NCAM-006 | The `SnapdragonFlight::MVCam` component shall allow starting and stopping of image taking | Unit Test
NAV-NCAM-007 | The `SnapdragonFlight::MVCam` component shall allow taking a single image | Unit Test



## 3. Design

### 3.1 Context

#### 3.1.1 Component Diagram

#### 3.1.2 Ports

The `SnapdragonFlight::MVCam` component uses the following port types:

Port Data Type | Name | Direction | Kind | Usage
-------------- | ---- | --------- | ---- | -----
[`Fw::BufferSend`](../../../Fw/Buffer/docs/sdd.html) | ImageSend | output | output | Port to publish captured images.

### 3.2 Functional Description

#### 3.2.1 Image Capture

The `SnapdragonFlight::MVCam` component uses the Qualcomm libcamera API to capture images from the
OV7251 optic flow camera on the Sunny MD102-A module.

### 3.3 Scenarios

`SnapdragonFlight::MVCam` has no significant scenarios.

### 3.4 State

`SnapdragonFlight::MVCam` has no state machines.

### 3.5 Algorithms

#### 3.5.1 Software Autoexposure

`SnapdragonFlight::MVCam` uses the Qualcomm Machine Vision SDK CPA (camera parameter
  adjustment) algorithm.

## 4. Dictionaries

[Dictionaries](NavCam.html)

## 5. Module Checklists

N/A

## 6. Unit Testing
The NavCam unit tests are designed to test interfaces and functionality with the available ports, command processing, telemetry output, EVR and data product generation.

## 6.1 Unit Test Descriptions

## 6.2 Unit Test Output

## 6.3 Unit Test Coverage

## 7. Change Log

Date | Description
---- | -----------
2/22/2019 | Port over from existing project
# Tello Visual Servoing Package for ROS 2 Foxy

The Tello Visual Servoing Package is designed to interact with the Tello drone using the Tello driver developed by Clyde McQueen. This package focuses on extracting information from the Tello’s camera feed. Leveraging Python scripts and OpenCV 4.9, it implements an Image Based Visual Servoing algorithm to track and follow an Aruco marker.

## Features
 - Tello Driver Integration: Communicate with the Tello drone via the provided driver.
 - Camera Feed Analysis: Extract relevant data from the Tello’s camera stream.
 - Visual Servoing: Implement a robust algorithm to follow an Aruco marker.
 - ROS 2 Foxy Compatibility: Designed for ROS 2 Foxy.

## Installation
 - Install ROS 2 Foxy (if not already done).
 - Clone this repository into your ROS workspace.
 - Build the package using colcon build.

## Usage

This package contains three main scripts. Below is a guide on what they do and how to run them:

### 1. Tello TakeOff and Land

```
ros2 run tello_ibvs takeoff_and_land
```

### 2. Tello Keyboard Teleop

```
ros2 run tello_ibvs teleop_keys
```

### 3. Tello Visual Servoing

```
ros2 run tello_ibvs visual_servoing
```
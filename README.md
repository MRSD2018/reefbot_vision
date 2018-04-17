# Reefbot Vision ROS Package

## Overview

Reefbot Vision ROS package to be used with the Videoray Pro 4, owned by the Reefbot Team, Field Robotics Center, Carnegie Mellon University.

The camera on the Videoray Pro4 has been replaced by an Allied Vision Technologies Manta G-201 machine vision camera.

This package runs in tandem with the apriltag_ros package, by publishing image topics and camera calibration values (yes, calibrate it first) over ros to get pose estimation of camera with respect to the apriltag detected, or vice-versa.

## How to run

Make sure the camera is set onto the same subnet as the computer, and the ROS workspace is ready.

```
cd catkin_ws/src/
git clone https://github.com/aaaaaaron/reefbot_vision.git
git clone https://github.com/RIVeR-Lab/apriltags_ros.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
roslaunch reefbot_vision pymba_apriltag.launch
```

Things should start popping up, the detection topics can be seen through rostopic echo. Visualize with Rviz.

## Work in progress

- Bounding boxes to apriltags
- Hector trajectory publishing
- Camera calibration information using parameter files instead

## Challenges

- Exposure time limits the frame rate, which makes the illumination within the tank critical.
- Network limitations caused by high resolution of camera, currently averted by routing ethernet cable to computers instead of through Wifi.
- Hardware access, taking the Reefbot apart to access the camera is tedious.

## Credits

- Pymba package, python wrapper for AVT cameras, https://github.com/morefigs/pymba
- Dan Arnett who introduced it to me

## Us

Awesome team of robotics graduate students under the MRSD, Robotic Systems Development Program, led by Oliver Kroemer, our Robot Autonomy professor and David Wettergreen our advisor for the project.


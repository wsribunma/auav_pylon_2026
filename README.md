# PURT Autonomous Fixed-Wing UAV Pylon Racing Competition 2025-2026

* Purdue Center for AI in Digital, Autonomous, and Augmented Aviation (AIDA^3)
* Purdue UAS Research and Test Facility (PURT)

## Overview
The Autonomous Fixed-Wing UAS Pylon Racing Competition is hosted by Purdue’s Center for AI in Digital, Autonomous, and Augmented Aviation (AIDA3) in collaboration with Purdue UAS Research and Test Facility (PURT) and the Rosen Center for Advanced Computing (RCAC). Running about 2 months, the challenge emphasizes learning and rapid deployment: newcomers to robotics and Guidance, Navigation, and Control (GNC) are encouraged. Teams design, integrate, and flight-test autonomy to safely fly fixed-wing aircraft around a pylon course. Further, this challenge focuses on a really important topic of autonomy in aerial vehicles!

## Task
Advanced Air Mobility (AAM) and evolving rulemaking (e.g., the proposed Part 108) are redefining aviation. As national restrictions ease and industry looks toward autonomous operations, the demand for safe, capable autonomy is accelerating.

Your challenge is to design and implement a Guidance, Navigation, and Control (GNC) method that enables a fixed-wing UAS to safely and swiftly navigate a pylon course and post the fastest valid lap.
* **Test Site:** 180 ft x 100 ft at PURT (indoor)
* **Ceiling:** 30 ft AGL
* **Provided data:** millimeter-accuracy 6-Degree-of-Freedom (6-DoF) motion capture data (position + orientation) at 100 Hz 

## Submission
Teams are asked to create GNC packages in Robot Operating Software 2 (ROS2). Please submit a fork version-controlled for this repository containing your solution script. (Details TBA)

## The Sample Solution Overview

The Python-based controller script is organized as follows:
```
sim_recs_ros_xtrack.py
┗ cross_tracker.py
┗ tecs_xtrack_sample.py
```

The sample solution uses Total Energy Control System (TECS), a cascaded propotional-integral controller, to control the vehicle's airborne states and a form of cross-track minimization navigation algorhm, the sample solution allows autonomous fixed-wing vehicle flights.

The sample solution uses Robot Operating Software (ROS2) with Python interface to perform high-fidelity data processing and computation to control the vehicle at PURT. The gain tuning given in the sample solution provides a guide-line for the general trim for the vehicle.

## Running The Simulator

This sample solution is tested on **Ubuntu 24.04** running **ROS2 Jazzy** version.

First install dependencies and install simulator. This is automatically installed when you install Cognpilot from main branch and recently pulled the "cyecca" branch.

To install the simulator alone in a single workspace:

```
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/CogniPilot/cyecca.git
git clone https://github.com/wsribunma/auav_pylon_2026.git
git clone https://github.com/wsribunma/cub_description.git 
cd ..
colcon build --symlink-install
```

Source the workspace
```
source install/setup.bash
```

Running the simulator
```
ros2 launch auav_pylon_2026 fixedwing_sim.xml
```
The launch script should launch a simulation with RVIZ2 visualizer by default.

The real-time simulation can be viewed from RVIZ2 or other ROS2 topic visualizer. 

Current simulation allows an optional use of a joystick controller (Logitech f310). The "A" button will toggle the auto model, and "B" button will toggle the manual mode. To enable joystick commands, add a condition ```use_joystick:=true``` when launching the fixedwing_sim.xml script


## Common Issues
If your simulator doesn't automatically takeoff, it may be that you are on "manual" mode. 
* Fix: go to your "ros2_ws/src/cyecca/scripts/fixedwing_sim.py" and set ```self.input_mode = "auto"```


*Note* By default, the sample solution is set to take off when below a given altitude to force the fixed-wing simulator to take off from the ground.

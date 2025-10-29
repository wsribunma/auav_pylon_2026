# PURT Autonomous Fixed-Wing UAV Pylon Racing Competition 2025-2026

* Purdue Center for AI in Digital, Autonomous, and Augmented Aviation (AIDA^3)
* Purdue UAS Research and Test Facility (PURT)
* Rosen Center for Advanced Computing (RCAC)

### **In-person PURT Flight:** December 06, 2025

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

The real-time simulation can be viewed from RVIZ2 or another ROS2 topic visualizer. 

The current simulation allows for the optional use of a joystick controller (Logitech f310). The "A" button will toggle the auto model, and "B" button will toggle the manual mode. To enable joystick commands, add a condition ```use_joystick:=true``` when launching the fixedwing_sim.xml script

## ROS2 Topics & Message Format

| Topic name        | Message type               | Array size | Element order (index)                      | Accepted ranges / format                                                                 | Notes                                                                                              |
|-------------------|----------------------------|------------|--------------------------------------------|-------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------|
| `/sim/auto_joy`   | `sensor_msgs/msg/Joy` | 5          | `[0]=Aileron, [1]=Elevator, [2]=Throttle, [3]=Rudder, [4]=Mode` | Aileron/Elevator/Rudder: `[-1.0, +1.0]` • Throttle: `[0.0, 1.0]` • Mode: `{-1.0, 1.0}` | Default order **AETR + Mode**. This is used for Auto mode. |
| `/sim/joy`   | `sensor_msgs/msg/Joy` | N/A          | `[0]=Aileron, [1]=Elevator, [2]=Throttle, [3]=Rudder, [4]=Mode` | Aileron/Elevator/Rudder: `[-1.0, +1.0]` • Throttle: `[0.0, 1.0]` • Mode: `{-1.0, 1.0}` | Default order **AETR + Mode**. This is used for Manual mode. |
| `/uav/joy_serial_status`   | `std_msgs/Float32MultiArray` | 5          | `[Aileron, Elevator, Throttle, Rudder, Mode]` | Aileron/Elevator/Throttle/Rudder: `[1000,2000]` | Set of 5 PWM signals to be parsed through serial port to Arduino|
| `/sim/pose` | `geometry_msgs/msg/Pose.msg` | N/A | `position, orientation`| N/A | Body-frame position relative to world frame, composed with position and quaternion-coordinate orientation |
| `/sim/odom` | `nav_msgs/msg/Odometry.msg` | N/A | `pose, twist`| N/A | Estimate of position and veloicty in free space |


## Input Commands (AETR)
In the simulator, Joy messages are used for controlling the vehicle; the Joy-to-PWM mappings apply only during the hardware integration. The "Mode" switch does not apply to the simulator. It will only take effect on board a real vehicle.


| Index | Channel   | Description                                        | ROS value → PPM (μs) mapping                            | Notes                                                                                 |
|------:|-----------|----------------------------------------------------|----------------------------------------------------------|---------------------------------------------------------------------------------------|
| 0     | Aileron   | Roll command                                       | `-1.0 → 1000`, `0.0 → 1500`, `+1.0 → 2000`               | Positive Joy (>1500 μs) = roll left; Negative Joy (<1500 μs>)= roll right                                           |
| 1     | Elevator  | Pitch command                                      | `-1.0 → 2000`, `0.0 → 1500`, `+1.0 → 1000`               | Positive Joy (<1500 μs) = Elevator up (pitch down); Negative Joy (>1500 μs) = Elevator down (pitch up) |
| 2     | Throttle  | Throttle                                           | `0.0 → 1000`, `1.0 → 2000`                               | Zero Joy (1000 μs) = Throttle idle; Joy 1.0 (2000 μs) = Full Throttle                                                             |
| 3     | Rudder    | Yaw command                                        | `-1.0 → 1000`, `0.0 → 1500`, `+1.0 → 2000`               | Positive Joy (>1500 μs)= yaw left; Nagative Joy (<1500 μs) = yaw right                                                                 |
| 4     | Mode      | Flight mode (discrete)                             | `-1.0 → 1000` (Manual), `1.0 → 2000` (Stabilize)          | Values are treated as discrete states: 1000 = Manual, 2000 = Stabilized|


## Common Issues
If your simulator doesn't automatically takeoff, it may be that you are on "manual" mode. 
* Fix: go to your "ros2_ws/src/cyecca/scripts/fixedwing_sim.py" and set ```self.input_mode = "auto"```


*Note* By default, the sample solution is set to take off when below a given altitude to force the fixed-wing simulator to take off from the ground.

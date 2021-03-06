# G-BEAM (Graph-Based Exploration And Mapping) ROS packages

This repository contains the ROS packages developed and used for simulation and experimental tests of G-Beam controller.
The control algorithm is described in the article **insert article here**.

## Test videos

### G-BEAM experimental Tests, ROS + Quadcopter Drone
[![gbeam experimental test](img/0suE8IxzbC0.png)](https://www.youtube.com/watch?v=0suE8IxzbC0)

### G-BEAM simulation on MatLab/Simulink
[![gbeam matlab simulation](img/9D0L84BI0Cg.png)](https://www.youtube.com/watch?v=9D0L84BI0Cg)

## Scheme of the packages
![scheme of gbeam package](img/11_gbeam_scheme.png "packages scheme")
This scheme represents the interactions between the packages:
Package `s1000_interface` is used to interact with the lower level controller of the drone (a DJI S1000, controlled with a DJI A3 board) and the LiDAR sensor.
Package `gbeam_controller` is where the control algorithm is actually implemented.
Package `gbeam_ground` contains the nodes that allow the visualization of the data from the other packages, it reads the topics with custom messages, and writes on topics with standard messages to be shown in `rviz`.
Package `gbeam_library` contains message definitions and functions used by the other packages.


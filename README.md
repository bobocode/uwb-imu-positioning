# uwb-imu-positioning
## Tightly-coupled integrated navigation system between UWB and IMU for UAV indoor localization
### This is C++ version code of the paper [Tightly coupled integrated navigation system via factor graph for UAV indoor localization](https://www.sciencedirect.com/science/article/pii/S127096382031052X) 
### Currently it provides loosely and tightly coupled FGO integration. More codes is coming....

#### Introduction
A ROS based library to perform localization for robot swarms using Ultra Wide Band (UWB) and Inertial Measurement Unit (UWB).

#### Features
- [x] beacon positioning with only UWB.
- [x] FGO localization algorithms that integrate UWB and IMU.
- [ ] EKF localization algotithms that integrate UWB and IMU. **Coming...**

### 1. Prerequisites
#### 1.1 Ubuntu and ROS
   * Ubuntu 64-bit 18.04
   * ROS Melodic. [ROS INSTALLATION](http://wiki.ros.org/ROS/Installation)
   
#### 1.2 Ceres Solver
   * Ceres. [Ceres INSTALLATION](http://ceres-solver.org/installation.html)



#### Acknowledge
The Code is modified from the work done by [Vins-mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) and [uwb-localization](https://github.com/lijx10/uwb-localization)

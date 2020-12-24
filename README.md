# AGV & ROS Navigation Stack
**Author**: Shimin Pan\
**Place**: ZJU (Mechanical Engineering Dept.)\
**Time**: April 2019

- [AGV & ROS Navigation Stack](#agv--ros-navigation-stack)
  - [1. Overview](#1-overview)
  - [2. Hardware & Software](#2-hardware--software)
  - [3. Details](#3-details)

## 1. Overview
![SLAM_Sketch](https://raw.githubusercontent.com/InspireFish/ZJU_ROS_AGV/master/markdown/slam.png)

## 2. Hardware & Software
* Automated Guided Vehicle(AGV)
* SLAMTEC inc. RPLIDAR A2 lidar (indoor, max range of 16m, 10Hz)
* UART serial port, CP2102 module
* SLAM algorithm: Rao-Blackwellization Particle Filter (RBPF)
* Augmented Monte Carlo sampling localization (in AMCL package)
* Global path planning: Dijkstra path planning (in navfn package)
* Local path planning: Dynamic Window Approach (DWA, in base_local_planner)

## 3. Details
* The way to [deploy (Chinese version only)](怎样使用我的毕业设计.docx) ROS packages
* The report about this [project (Chinese version only)](毕业论文_潘世民.docx)
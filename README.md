# ROS-RRT

Implement RRT (rapidly exploring random tree) in ROS

## Preparation

1. Install Ubuntu and ROS (This project is tested on Ubuntu14.04 and ROS indigo)
2. Create your *workspace* folder

## nav_sim

This is the test environment for visualization

Clone it to your *workspace/src* folder

Thanks to YinHuan@ZJU and his [repo](https://github.com/ZJUYH/nav_sim)

## RRT

1. Download *[navigation](https://github.com/ros-planning/navigation)* source code to your repo 

   NOTE: The branch of [navigation](https://github.com/ros-planning/navigation) should match your ROS version

2. Replace the *global_planner* using this repo’s file

3. If errors come out while *catkin_make*, you’d better read my code and modify it yourself, I mainly add *rrt* and modified *traceback* & *planner_core*

 
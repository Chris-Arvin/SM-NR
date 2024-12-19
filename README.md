# SM-NR
This is the source code of the SM-NR [**SM-NR: Scene Modeling of Autonomous Vehicles Avoiding Stationary and Moving Vehicles on Narrow Roads**](https://arxiv.org/abs/2412.13305), an approach focused on the navigation on narrow roads.
[![](https://github.com/user-attachments/assets/530769a5-7a3b-436d-a179-2167a4841c06)](https://www.youtube.com/watch?v=9uhLzXsn28w)


## Table of Contents
* [Installation](#1-Installation)
* [Quick Start](#2-Quick-Start)
* [Introduction to use](#3-Introduction-to-Use)
* [Contributors](#4-Contributors)
* [Acknowledgement](#5-Acknowledge)


## 1. Installation
The project has been tested on Ubuntu 18.04 (ROS Melodic). To install the repository: 
```
$ mkdir -p SMNR_ws/src
$ cd SMNR_ws/src
$ git clone https://github.com/Chris-Arvin/SM-NR.git
$ cd ..
$ rosdep install –from-paths src –ignore-src –rosdistro-melodic -y
$ catkin_make
```


## 2. Quick Start
To see the performance of SM-NR in the conflict scenario with two meeting gaps, open the first terminal to open the environment simulator: 
```
$ source SMNR_ws/devel/setup.bash
$ roslaunch vehicle_simulator vehicle_simulator.launch
```
Open the second terminal to open the SM-NR navigation: 
```
$ source SMNR_ws/devel/setup.bash
$ roslaunch move_base_bridge move_base_bridge.launch
```


## 3. Introduction to use

1. control the oncoming moving vehicle with the keyboard
2. select the goal of the autonomous vehicle by 2D_Nav_Goal


## 4. Contributors
* Qianyi Zhang  arvin.nkzqy@gmail.com
* Jinzheng Guang
* Zhenzhong Cao
* Jingtai Liu


## 5. Acknowledge
The motivation for this research on modeling narrow-road meeting scenarios originates from Qianyi Zhang’s internship experience in the Autonomous Delivery Robot Department at Meituan Group. We are grateful to the company for presenting these challenging cases and especially appreciate the guidance and insightful discussions with Zhang’s mentor, Xiao Li.


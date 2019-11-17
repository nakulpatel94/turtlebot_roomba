# turtlebot_roomba
Simple Walker algorithm for a robot like roomba vacuum cleaner 


[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# ROS Turtlebot_roomba
This repository contains simple walker algorithm for Turtlebot much like Roomba vacuum cleaner. The robot used for this purpose is Turtlebot and the execution has been showm with simulation in Gazebo. The turtlebot moves forward until it reaches an obstacle (but not colliding), and then rotates in place until the way ahead of it is clear, and then again starts moving forward. Thus, it exibits obstacle avoidance behaviour.

Note: The minimum distance for detecting the obstacle can be varied.

It contains following main files:
- include/walker.hpp
- launch/turtlebot_walker.launch
- src/main.cpp
- src/walker.cpp

## Dependencies
For this project, you require following dependencies:

- Ubuntu 16.04
- ROS kinetic
- catkin
- Turtlebot packages

ROS can be installed from the http://wiki.ros.org site. Click on following link [here](http://wiki.ros.org/kinetic/Installation) to navigate to the installation guide for ROS.

To install Turtlebot packages, execute following command in terminal:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers

```

## Building this package(turtlebot_roomba)
To build the given package, first you need to create a catkin workspace, the steps of which are as below:
```
mkdir -p ~/your_workspace/src
cd ~/your_workspace/
catkin_make
source devel/setup.bash
```
After making a catkin workspace and building the workspace using catkin_make, you now need to follow these below steps to install and build the package:

```
cd src/
git clone --recursive https://github.com/nakulpatel94/turtlebot_roomba.git
cd ..
catkin_make
```
In above steps, we have again executed catkin_make command to rebuild our workspace with the freshly installed package(turtlebot_roomba)


## Running this package(turtlebot_roomba)
After successfully building the workspace with the package, we have already created the executables for our nodes:

- turtlebot_roomba is the executable for walker.cpp

Now, first step is to run roscore to start the ROS system

1. In a new terminal, execute roscore command as follows:

```
roscore
```

Next, ensure that you source your workspace using following command:

2. In a new teminal, navigate to your workspace and source the setup.bash file:
```
cd ~/your_workspace/
source devel/setup.bash
```
Now, to run the node, using roslaunch command as follows:

3. Execute following command:
```
roslaunch turtlebot_roomba turtlebot_walker.launch
```



## Using rqt_console for checking log messages

The following command can be used to see the GUI indicating various logger levels used in the nodes.

```
rqt_console
```


## Running bag files recorded as .bag

Bag files are used to record the informations and messages on various topics in the running ROS system.

To record a bag file using the roslaunch command and the enableBag argument(default is false) for enabling/disabling rosbag recording:
```
roslaunch turtlebot_roomba turtlebot_walker.launch enableBag:=true
```



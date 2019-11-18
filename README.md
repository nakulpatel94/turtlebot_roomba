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

ROS can be installed from the https://wiki.ros.org site. Click on following link [here](https://wiki.ros.org/kinetic/Installation) to navigate to the installation guide for ROS.

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

To play this bag file, simply type following commands in a terminal, after starting roscore:
```
cd ~/your_workspace/src/turtlebot_roomba/results
rosbag play rosbagRecorded.bag
```

To verify what all topics have been recorded using rosbag, simply execute following command in terminal:

Note: Run the command from results subdirectory

```
rosbag info rosbagRecorded.bag
```

This will yield following output:

```
path:        rosbagRecorded.bag
version:     2.0
duration:    19.9s
start:       Dec 31 1969 19:00:00.23 (0.23)
end:         Dec 31 1969 19:00:20.15 (20.15)
size:        7.9 MB
messages:    15321
compression: none [11/11 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            1993 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               1978 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              1978 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /gazebo_gui/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo_gui/parameter_updates                        1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     1979 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                     36 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                     199 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     1989 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                   72 msgs    : bond/Status                           (3 connections)
             /odom                                             1986 msgs    : nav_msgs/Odometry                    
             /rosout                                            225 msgs    : rosgraph_msgs/Log                     (9 connections)
             /rosout_agg                                        209 msgs    : rosgraph_msgs/Log                    
             /scan                                              183 msgs    : sensor_msgs/LaserScan                
             /tf                                               2484 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage
```


Also, we can see the topic for velocity specifically:
```
rostopic echo /mobile_base/commands/velocity

```


## References:

- https://wiki.ros.org/
- https://answers.ros.org/question/317168/how-can-i-use-the-data-from-the-laserscanmsg-inside-the-callback-function/




[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)
# ROS Walk Robot 

---

## Overview
This project creates a ROS node in order to simulate the Roommba vacum cleaner behavior in a turtlebot. This Robot has the goal of moving inside a room while avoiding collision with any obstacle.
This obstacle avoindance, is possible because turtlebot has a laser sensor that can sensor objects. when an obstacle is detected, an angular velocity is applied to the robot in order to make the robot to turn until it does not sense any obstaccle at his range.
Another funcionalitty of this project, is the possibility of recording the ROS topics into a rosbag using the launch file created to launch the project.
In this rosbag is recorded all topics except /camera/*. 

## License

BSD License
Copyright 2019 Andre Ferreira

```
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

```


## Dependencies




### Install ROS kinetic

In order to Install ROS kinect follow the following ROS.org [link](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### Install catkin

In order to Install catkin follow the following ROS.org [link](http://wiki.ros.org/catkin#Installing_catkin)

### Install Turtlebot packages

```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers

```

## How to build

Create and build a catkin workspace 
```
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/
 catkin_make
 
```
Build Project in Catkin Workspace
```
 cd ~/catkin_ws/
 source devel/setup.bash
 cd src
 git clone https://github.com/andrelgf1/walk_robot.git
 cd ~/catkin_ws/
 catkin_make
```
## How to run node using launch file

In order to Run the node, the map and the turtlebot at the same time the launch file can be used 

There is no need to initialize the master.

Inside catkin workspace

```
source devel/setup.bash
roslaunch walk_robot  walk_robot.launch

```

## How to run each node separately

once your environment is set

open the terminal and initialize the master
 
```
roscore

```
In a new terminal, lets run the publisher talker

```
 cd ~/catkin_ws/
 source devel/setup.bash
 rosrun beginner_tutorials talker

```
In a new terminal, lets run the Subscriber listener
```
 cd ~/catkin_ws/
 source devel/setup.bash
 rosrun beginner_tutorials listener

```



## Recording bag files with the launch file

This section uses the launch file to activate the recording of  the topic data from a running ROS system, and will accumulate the data in a bag file.
The bag file called recorded.bag will be saved at the "results" folder.
In order to record the topics of the system, run the launch file passing the argument "enable".
The recording does not contain information about /camera/*.

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch walk_robot  walk_robot.launch record:=enable

```

If it is not desired recording the topics just Run the launch file without the argument record

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch walk_robot  walk_robot.launch  

```
or 

pass "disable" to the argument record 

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch walk_robot  walk_robot.launch record:=disable

```

## Inspecting the bag file

To inspect the recorded.bag , to check the details of what was recorded.

```
cd ~/catkin_ws
cd src/walk_robot/results/
rosbag info recorded.bag

```

## Playing the recorded bag

To play the recorded bag 

```
roscore

```
In a new terminal 

```
cd ~/catkin_ws
cd src/walk_robot/results/
rosbag play recorded.bag

```
In order to check the recorded bag, it's possible to echo the topic to veriry the velocities being published.

```
rostopic echo /mobile_base/commands/velocity

```










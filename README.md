## Capstone Project Team DrivingCarla

### Teammembers:

* Sven-Garrit Czarnian ([Sven-Garrit.Czarnian@hella.com](Sven-Garrit.Czarnian@hella.com))
* Tina Wein ([tina.wein@hella.com](tina.wein@hella.com))
* Thomas Götz ([Thomas.Goetz@tgoetz.net](Thomas.Goetz@tgoetz.net))
* Jyoti Nayak ([jyotinayak1976@gmail.com](jyotinayak1976@gmail.com))
 
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. 

## Overview
### Final Project Architecture
The following is our system architecture diagram showing the ROS nodes and topics used in our final project. The ROS nodes and topics shown in the diagram are described briefly in the component chapters below.

![](imgs\architecture.png)
![](https://github.com/carndsven/carnd_final_project/blob/master/imgs/final-project-architecture.png)

It includes traffic light detection, control and waypoint following and this is the core functionality of the autonomous vehilce system.

Differing to the provided system architecture by Udacity we add an additional channel between the waypoint updater and the dbw node `gap_to_trajectory` to provide the distance to the trajectory for the control and the curvature for the pilot control.

#### Way-Point Updater Node
The code for the waypoint updater can be found under [/ros/src/waypoint_updater/](/ros/src/waypoint_updater/). The purpose of the waypoint updater node is to provide a fixed number of waypoints with correct target velocitys depending on the given obstacle data and traffic light information. The inputs of the node are  `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint`. It publishs a list of waypoints ahead of the car with target velocities to the `/final_waypoints topic` and distance to the trajectory data to the `gap_to_trajectory topic`. 

![](imgs\waypoint_updater_node.png)
![](https://github.com/carndsven/carnd_final_project/blob/master/imgs/waypoint_updater_node.png)

In case of an detected red traffic light the waypoint updater adjust the speed of the car to perform a smooth and save stop in front of the traffic light.

[TBD] Images for result of waypoint updater node: 1 image without and one with traffic light insight

#### DBW Node Node
The code for the drive by wire control can be found under [/ros/src/twist_controller/](/ros/src/twist_controller/). It consists of the files for the dbw node `dbw_node.py` and the `twist_controller.py`, which includes a PID and lowpass filter.
The dbw node will publish throttle, steering and brake commands to the `vehicle/throttle_cmd`,`vehicle/steering_cmd` and `vehicle/brake_cmd` topics. It subribes to the `/current_velocity` topic, the `/twist_cmd` topic (includes target linear and angular velocity), the  `/vehicle/dbw_enabled` topic (indicated if the car is under autonomus drive by wire control or under manual driver control) and the `gap_to_trajectory` topic (includes distance information to the trajectory).

![](imgs\dbw_node.png)
![](https://github.com/carndsven/carnd_final_project/blob/master/imgs/dbw_node.png)

The steering controller is a combination of a feed forward control and a distance controler.
The dbw status will be taken into account to reset the controller if the safety driver takes over the control to avoid accumulation errors in the controller.

[TBD] Example image with slight difference between planned and current trajectory.

#### Traffic Light Detection Node

[TBD]

![](imgs\traffic_light_detection_node.png)
![](https://github.com/carndsven/carnd_final_project/blob/master/imgs/traffic_light_detection_node.png)

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

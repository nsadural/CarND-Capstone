# Programming a Real Self-Driving Car

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the starting project [here](https://github.com/udacity/CarND-Capstone).

Project Team:
 1. Team Lead - Felix Larsen (i4guar@gmail.com)
 2. Team Member - Xiaohui Luo (xiaohuil@umich.edu)
 3. Team Member - Nikko Sadural (nsadural@purdue.edu)

As a team our tasks were to:
* Select the correct waypoints and determine the target velocities at them depending on the traffic light situation
* Publish the steering, brake and gas input to the Drive-By-Wire module
* Detect the traffic lights and recognize their color to be able to react accordingly 

## Simulator
[//]: # (Image References)

[follow_road]: ./res/follow_road.png "Follow Road"
[green_light_continue]: ./res/green_light_continue.png "Green Light Continue"
[red_light]: ./res/red_light.png "Red Light"
[simulator]: ./res/simulator.png "Simulator"
[traffic_light_sim]: ./res/traffic_light_sim.png "Traffic Light Simulator"
[traffic_light_real]: ./res/traffic_light_real.png "Traffic Light Real World"

This is the simulator provided by udacity:
![alt text][simulator]

The car is able to stop at red lights:
![alt text][red_light]

It will continue when the traffic lights are green:
![alt text][green_light_continue]

The car also follows the lane of the road:
![alt text][follow_road]

This is a short [screen recording of the result](./res/video.mov).

## Traffic light detection

The car is able to recognize traffic lights in both a real environment and the simulator:
![alt text][traffic_light_sim]
![alt text][traffic_light_real]

## Installation

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

1. `git clone` this project repository

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

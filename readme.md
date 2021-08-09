# Hornet 7 Assignment 2

This repository contains a ROS package called "control_toy". This consists of a simulator that is similar to what was shown. Your job is to implement a PID controller to control the orange box (we'll affectionately call it a rocket) in the simulator.

DO NOT MODIFY "simulator.py" (but feel free to take a peek)

Zip and email your solution to grajiv@u.nus.edu

## Getting the simulation up and running

In addition to ROS, this simulation depends on pygame. to install pygame on ubuntu you have two options. Either run:

```
sudo apt install python-pygame
```

or

```
pip install pygame (ROS Melodic)
pip3 install pygame (ROS Noetic)
```

To run the simulator add `control_toy` into your catkin workspace. Build the catkin workspace then run:

```
rosrun control_toy simulator.py
```

Run the controller separately on another terminal with:

```
rosrun control_toy myPIDController.py
```

## Using the simulator

The simulator publishes the following data in `std_msgs/Float64` format:

```
/simulator/depth
/simulator/setpoint
```

The simulator subscribes to the following topic (also of `std_msgs/Float64` type). 

```
/simulator/thruster
```
Publishing to this topic will allow you to control the vertical thruster on the rocket.

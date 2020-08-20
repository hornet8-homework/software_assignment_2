# Hornet Homework 2

This is repository contains a ros package called "control_toy". This consists of a simulator that is similar to what was shown. Your job is to design a control algorithm for this simulator.

DO NOT MODIFY "simulator.py".

Zip and email your solution to rrkarthigeyan@u.nus.edu

## Getting the simulation up and running

In addition to ROS, this simulation depends on pygame. to install pygame on ubuntu you have two options. Either run:

```
sudo apt install python-pygame
```

or

```
pip install pygame
```

To run the simulator add the package to your catkin workspace. Build the catkin workspace then run:

```
rosrun control_toy simulator.py
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
Publishing to this topic will allow you to control the vertical thruster on the orange box.

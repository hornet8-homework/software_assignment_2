# Hornet 8.0 Assignment 2

This repository contains a ROS package called "control_toy". This consists of a simulator that is similar to what was shown. Your job is to implement a 1D PID controller to control the orange box (we'll affectionately call it a rocket) in the simulator.

DO NOT MODIFY "simulator.py" (but feel free to take a peek)

Zip and email your solution (just your controller package named `yournusnetid_controller`) to [hornet8.0homework@gmail.com](mailto:hornet8.0homework@gmail.com)

## Getting the simulation up and running

In addition to ROS, this simulation depends on pygame. to install pygame on ubuntu you have two options. Either run:

```bash
sudo apt install python-pygame
```

or

```bash
pip install pygame (ROS Melodic)
pip3 install pygame (ROS Noetic)
```

To run the simulator, add `control_toy` to your workspace, build the catkin workspace then run:

```bash
rosrun control_toy simulator.py
```


```bash
rosrun your_controller yourPIDController.py
```
Refer to the control_toy readme for details about using the simulator.

A sample controller package with a sample python script has been included for your reference. Create your controller package named `yournusnetid_controller`, e.g. eXXXXXXX_controller
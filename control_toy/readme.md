# Control Toy

This repository contains a ROS package called "control_toy".

## Getting the simulation up and running

In addition to ROS, this simulation depends on pygame. to install pygame on ubuntu you have two options. Either run:

```bash
sudo apt install python-pygame
```

or

```bash
pip install pygame # (ROS Melodic)
pip3 install pygame # (ROS Noetic)
```

To run the simulator, build the `control_toy` package, then run:

```bash
rosrun control_toy simulator.py __ns:=rocket
```

This will start the simulation containing the vehicle, and create the pygame gui.

## Using the simulator

The simulator publishes the following data in `std_msgs/Float64` format:

```bash
/rocket/depth    # current vehicle depth
/rocket/setpoint # current setpoint
```

The simulator subscribes to the following topic (also of `std_msgs/Float64` type). 

```bash
/rocket/thruster # -4 to 4, with upwards positive.
```
Publishing to this topic will allow you to control the vertical thruster on the rocket.

You can update the setpoint by sending a `UpdateSetpointRequest` ROS service request to `/rocket/update_setpoint`, or left-clicking on anywhere in the gui.
# Control Toy

This repository contains a ROS package called "control_toy".

## Getting the simulation up and running

In addition to ROS, this simulation depends on pygame to display the gui.
Enabling the pygame gui is optional but useful for debugging.
To install pygame on ubuntu, run any one of the following lines:

```bash
sudo apt install python-pygame
pip install pygame # (ROS Melodic)
pip3 install pygame # (ROS Noetic)
```

To run the simulator, build the `control_toy` package, then run:

```bash
rosrun control_toy simulator.py _vehicle:=rocket _gui:=true
```

This will start the simulation containing the vehicle, and create the pygame gui if _gui:=true.

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
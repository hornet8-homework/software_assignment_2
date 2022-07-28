#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

"""
You need to complete the following 3 steps to get this script to work:
1. Add Missing Subscribers
2. Implement your PID Controller
3. Tune your PID Controller
"""

# declaration of global variables
global depth
global setpoint
setpoint = 0
depth = 0
thrust = 0

# Declaration of callback functions


def SetPointCallback(msg):
    global setpoint
    setpoint = msg.data


def PIDControlCallback(msg):
    global depth
    depth = msg.data


# Main
if __name__ == '__main__':
    rospy.init_node('PID_Controller')
    pub_thrust = rospy.Publisher('/rocket/thruster', Float64, queue_size=10)
    """
    ==================================================================================
    TODO: Implement your Subscribers to '/rocket/setpoint' & '/rocket/depth' here.
    Hint: They are 'Float64' type.
    ==================================================================================
    """
    # Step 1: Add Missing Subscribers here.

    rospy.loginfo("Time to rock and roll")

    """
    Our assumption: Distance is measured in metres from the surface and our update interval is a constant 50 ms.
    The thruster topic takes in the desired acceleration upwards from -4 m/s^2 to 4 m/s^2. (the vehicle has a mass of 1 kg)
    """
    # Step 3: Tune your PID Controller
    KP = 0  # What Proportional value is good?
    KI = 0  # What integral value is good?
    KD = 0  # What differential value is good?
    bias = 0  # Is a bias necessary?

    # Declare all the variables that you need here!
    iteration_time = 0.05  # Assume constant update intervals.

    while not rospy.is_shutdown():

        """
        ==================================================================================
        TODO: Implement your PID Control code here.
        Hint: You may need to declare a few more variables.
        The result of your PID Controller should be stored in the variable named "thrust".
        Note that a +ve thrust is upwards while -ve thrust is downwards.
        ==================================================================================
        """
        # Step 2: Add your PID Controller code here.

        """
        Since the rocket is positively buoyant, we do not need to worry about moving upwards.
        To avoid wearing out the hypothetical thruster and save energy,
        we can stop the motor instead of propelling the equipment upwards.
        """
        # thrust = min(thrust, 0)

        pub_thrust.publish(Float64(thrust))
        rospy.sleep(iteration_time)

    pub_thrust.publish(Float64(0))  # Stop vehicle before exiting
    rospy.loginfo("Shutting Down PID Controller...")

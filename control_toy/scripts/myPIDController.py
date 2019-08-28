#!/usr/bin/env python
#Ensure you are using the correct python version.

import time
import rospy
from std_msgs.msg import Float64, Bool

"""
You need to complete the following 3 steps to get this script to work:
1. Add Missing Subscribers
2. Implement your PID Controller
3. Tune your PID Controller
"""

#declaration of global variables
global depth
global setpoint
setpoint = 0
depth = 0

#Declaration of callback functions
def SetPointCallback(msg):
    global setpoint
    setpoint = msg.data

def PIDControlCallback(msg):
    global depth
    depth = msg.data

#Main
if __name__ == '__main__':
    rospy.init_node('PID_Controller')
    pub_thrust = rospy.Publisher('/simulator/thruster', Float64, queue_size=10)
    """
    ==================================================================================
    TODO: Implement your Subscriber to '/simulator/setpoint' & '/simulator/depth' here.
    Hint: They are 'Float64' type.
    ==================================================================================
    """
    #Step 1: Add Missing Subscribers here.
    
    print("Time to rock and roll")

    """
    Our assumption: Distance is measured in metres and our update interval is a constant 50 ms.
    For fun, assume that the max speed of the thruster is 8 m/s (0.4 metres per 50 ms) and that
    our vehicle sinks at a constant 2 m/s (0.1 metres per 50 ms).
    """
    #Step 3: Tune your PID Controller
    KP = 0      #What Proportional value is good?
    KI = 0      #What integral value is good?
    KD = 0      #What differential value is good?
    bias = 0    #Is a bias necessary?

    #Declare all the variables that you need here!
    iteration_time = 0.05 #Assume constant update intervals.

    while not rospy.is_shutdown():

        """
        ==================================================================================
        TODO: Implement your PID Control code here.
        Hint: You may need to declare a few more variables.
        The result of your PID Controller should be stored in the variable named "thrust".
        Note that a +ve thrust is upwards while -ve thrust is downwards.
        ==================================================================================
        """
        #Step 2: Add your PID Controller code here.



        """
        Just for fun,
        Assume that the equipment is experiencing a constant acceleration due to gravity,
        hence, we do not need to worry about moving downwards.
        To avoid wearing out the hypothetical thruster and save energy,
        we can stop the motor instead of propelling the equipment downwards.
        """
        if thrust > 0.4:
            thrust = 0.4
        elif thrust < 0.0:
            thrust = 0.0

        pub_thrust.publish(Float64(thrust))
        print("Depth: {0:.4f} thrust: {1:.4f}".format(round(depth, 5), round(thrust, 5)))
        time.sleep(iteration_time)

    print("Shutting Down PID Controller...")
    print("Done")

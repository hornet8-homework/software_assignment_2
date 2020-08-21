#!/usr/bin/python
import sys
import time
import pygame
import rospy
from std_msgs.msg import Float64

#Global Variables
global thruster
depth = 0
velocity = 0
gravity = 0.1
setpoint = 100
tolerance = 1.0
display_size = (640,480)
thruster = 0

#Helpers
def constrain(val, lower, upper):
    return min(upper, max(lower, val))
def isWithin(val, lower, upper):
    return lower <= val and val <= upper

#Callbacks
def callback(msg):
    global thruster
    thruster = msg.data

#Main
if __name__ == '__main__':

    #Initialise Game
    pygame.init()
    screen = pygame.display.set_mode(display_size)
    pygame.display.set_caption('Hornet Assignment 2')

    #Setup ROS
    rospy.init_node("control_simulator")
    pub_depth = rospy.Publisher('/simulator/depth', Float64, queue_size=10)
    pub_setpoint = rospy.Publisher('/simulator/setpoint', Float64, queue_size=10)
    rospy.Subscriber("/simulator/thruster", Float64, callback)
    running = True

    #Primary loop
    while running:
        #Publish telemetry
        pub_depth.publish(Float64(depth))
        pub_setpoint.publish(Float64(setpoint))

        #Draw
        setPointLineColour = (46, 204, 113) if (abs(depth - setpoint) < tolerance) else (231, 76, 60)
        screen.fill((41, 128, 185))
        pygame.draw.rect(screen, (230, 126, 34), (310, depth, 20, 70), 0)
        pygame.draw.lines(screen, setPointLineColour, False,((0,setpoint), (display_size[0],setpoint)),3)
        pygame.display.update()

        #Business Logic
        thruster = constrain(thruster, -1.0, 1.0)
        velocity = (velocity + gravity - thruster) if isWithin(depth, 0, 400) else 0
        depth = constrain(depth + velocity, 0, 400)

        #Handle user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                setpoint = pos[1]

        #Sleep
        time.sleep(0.05)

    #Clean up
    pygame.quit()
    sys.exit()

#!/usr/bin/python
import pygame
import rospy
import sys
from std_msgs.msg import Float64
import time
depth = 0
velocity = 0
gravity = 0.1
setpoint = 100
global thruster
thruster = 0
def callback(msg):
    global thruster
    print "callback"
    thruster = msg.data
pygame.init()
screen = pygame.display.set_mode((640,480))
rospy.init_node("control_simulator")
pub = rospy.Publisher('/simulator/depth', Float64, queue_size=10)
setpoint_pub = rospy.Publisher('/simulator/setpoint', Float64, queue_size=10)
rospy.Subscriber("/simulator/thruster", Float64, callback)
running = True
while running:
    screen.fill((0,0,255))
    pub.publish(Float64(depth))
    setpoint_pub.publish(Float64(setpoint))
    pygame.draw.rect(screen, (255,255,0), (310, depth, 20, 70), 0)
    pygame.draw.lines(screen, (255,0,0), False,((0,setpoint), (640,setpoint)),3)
    pygame.display.update()
    if depth <= 400 and depth >= 0:
        velocity += gravity-thruster
        depth += velocity
    else:
        depth = 400 if depth > 400 else 0
        velocity = 0
        thruster  = 0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            setpoint = pos[1]
    time.sleep(0.05)
pygame.quit()
sys.exit()

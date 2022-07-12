#!/usr/bin/env python
import pygame
import rospy
from std_msgs.msg import Float64
import numpy as np

# Global Variables
global thruster

display_offset = 40
display_scale = 30
setpoint = 5
tolerance = 0.2
display_size = (250, 560)
thruster = 0

time_interval = 0.05

vehicle_width = 0.2
vehicle_height = 0.6
max_depth = 15

drag_coefficient = 4
vehicle_mass = 1
fluid_density = 41.6666666667 * 7.81 / 9.81
gravity = 9.81


class Motion:
    """Represent the 2d motion of the vehicle"""
    def __init__(self, x=0, y=0, vx=0, vy=0, drag_coeff=1.1, mass=1, fluid_density=997, gravity=9.81,
                 height=0.6, width=0.2, dt=0.05, env_width=8, env_depth=10, thrust_limit=2):

        self.drag_coeff = drag_coeff
        self.g = gravity
        self.fluid_density = fluid_density
        self.vehicle_dims = width, height
        self.mass = mass
        self.object_density = mass/(width * width * height)
        self.dt = dt
        self.X = np.array([x, y], dtype=np.float64)  # down / right +ve
        self.V = np.array([vx, vy], dtype=np.float64)
        self.env_constraints = np.array(
            [0, -5]), np.array([env_width, env_depth])
        self.thrust_limit = thrust_limit

    @staticmethod
    def constrain(val, lower, upper):
        return np.minimum(upper, np.maximum(lower, val))

    @staticmethod
    def isWithin(val, lower, upper):
        return lower <= val and val <= upper

    def volumeSubmerged(self):
        return (self.vehicle_dims[1] + min(self.X[1], 0)) * (self.vehicle_dims[0] ** 2)

    def update(self, horizontal_thrust=0, vertical_thrust=0):
        thrust = np.array([self.constrain(horizontal_thrust, -self.thrust_limit, self.thrust_limit),
                           self.constrain(vertical_thrust, -self.thrust_limit, self.thrust_limit)])
        Fb = np.array([0, -self.fluid_density *
                      self.g * self.volumeSubmerged()])

        Fg = np.array([0, self.mass * self.g])
        Fd = -0.5 * np.sign(self.V) * self.drag_coeff * np.array([self.vehicle_dims[0] * self.vehicle_dims[1], self.vehicle_dims[0] ** 2]) \
            * self.V ** 2
        F = Fb + Fg + Fd - thrust * self.mass
        newV = self.V + (F/self.mass * self.dt)
        newX = self.X + newV * self.dt
        newX = self.constrain(
            newX, self.env_constraints[0], self.env_constraints[1])
        newV *= (newX < self.env_constraints[1]) * \
            (newX > self.env_constraints[0])
        self.X = newX
        self.V = newV
        rospy.logdebug("Position: {} Velocity: {} Fb {} Fg {} Fd {} th {}".format(
            self.X, self.V, Fb, Fg, Fd, thrust))

    def getPosition(self):
        return self.X


motion = Motion(
    x=0, y=0, vx=0, vy=0, drag_coeff=drag_coefficient, mass=vehicle_mass, fluid_density=fluid_density,
    gravity=gravity, height=vehicle_height, width=vehicle_width, dt=time_interval, env_width=8, env_depth=max_depth,
    thrust_limit=10
)


def vertical_callback(msg):
    global thruster
    thruster = min(4, max(-4, msg.data))


def getY(depth):
    """Get the y position of a given depth in the screen"""
    return depth * display_scale + display_offset


# Main
if __name__ == '__main__':

    # Initialise Game
    pygame.init()
    screen = pygame.display.set_mode(display_size)
    pygame.display.set_caption('Hornet Assignment 2')

    # Setup ROS
    rospy.init_node("control_simulator", log_level=rospy.INFO)
    rospy.on_shutdown(pygame.quit)

    pub_depth = rospy.Publisher('/simulator/depth', Float64, queue_size=10)
    pub_setpoint = rospy.Publisher(
        '/simulator/setpoint', Float64, queue_size=10)
    rospy.Subscriber("/simulator/thruster",
                     Float64, vertical_callback)
    running = True
    surface = pygame.display.get_surface()
    start_time = rospy.get_time()
    count = 0

    # Primary loop
    while running and not rospy.is_shutdown():
        depth = motion.getPosition()[1]

        # Publish telemetry
        pub_depth.publish(Float64(depth))
        pub_setpoint.publish(Float64(setpoint))

        # Draw
        reached_goal = (abs(depth - setpoint) < tolerance)
        setPointLineColour = (46, 204, 113) if reached_goal else (231, 76, 60)
        if not reached_goal:
            count = 0
        else:
            count += 1
        if count == 40:
            rospy.loginfo("reached depth={:.2f} in {:.2f} seconds".format(setpoint, rospy.get_time() - start_time))

        screen.fill((0, 0, 0))
        w, h = surface.get_width(), surface.get_height()
        pygame.draw.rect(screen, (255, 255, 255),
                         (0, 0, w, getY(0)), 0)  # water surface
        pygame.draw.rect(screen, (41, 128, 185), (0, getY(0), w, getY(
            max_depth + vehicle_height) - display_offset), 0)  # water
        pygame.draw.rect(screen, (230, 126, 34), (w / 2, getY(depth),
                         vehicle_width * display_scale, vehicle_height * display_scale), 0)
        pygame.draw.lines(screen, setPointLineColour, False,
                          ((0, getY(setpoint)), (w, getY(setpoint))), 3)
        pygame.display.update()

        # Business Logic
        motion.update(vertical_thrust=thruster)

        # Handle user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                if event.button == 3: # Right click to update vehicle position                    
                    motion.X = np.array(
                        [pos[0] / display_scale, (pos[1] - display_offset) / display_scale])
                    motion.V = np.array([0, 0])
                else: # Left click to update setpoint
                    setpoint = Motion.constrain(
                        (pos[1] - display_offset) / display_scale, 0, max_depth)
                start_time = rospy.get_time()
                count = 0

        # Sleep
        rospy.sleep(time_interval)

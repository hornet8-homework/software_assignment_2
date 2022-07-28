#!/usr/bin/env python3
import pygame
import rospy
import os
import cv2
import random
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from control_toy.srv import UpdateSetpoint, UpdateSetpointResponse

# Global Variables
global thruster

time_interval = 0.05

display_offset = 40
display_scale = 30
setpoint = 5
tolerance = 0.2
display_size = (250, 560)
thruster = 0
vehicle_width = 0.2
vehicle_height = 0.6
max_depth = 15
drag_coefficient = 3
vehicle_mass = 1
gravity = 9.81
fluid_density = 41.6666666667 * (gravity + 2) / gravity


camera_topic = "camera/image_raw"
depth_topic = "depth"
thruster_topic = "thruster"
setpoint_topic = "setpoint"
setpoint_service_name = "update_setpoint"


class Camera:
    def __init__(self):
        """Simulates the camera of the rocket."""
        file_path = os.path.realpath(os.path.dirname(__file__))
        img = cv2.imread(file_path + "/../bg.jpg")
        self.tw, self.th = 200, 100
        bg = np.zeros(
            (img.shape[0] + self.th, img.shape[1], 3), dtype=np.uint8)
        bg[self.th:, :, :] = img
        self.h, self.w = img.shape[:2]

        intervals = 5
        coords = [(int(random.randint(int(self.w/2 - self.tw/4), int(self.w/2 + self.tw/4))),
                   int(self.h / (intervals + 1) * (i + 1) + self.th)) for i in range(intervals)]
        random.shuffle(coords)
        overlay = np.ones(bg.shape, dtype=np.uint8)
        cv2.circle(overlay, coords[0], random.randint(
            12, 30), (0, 30, 180), -1)
        cv2.circle(overlay, coords[1], random.randint(
            12, 30), (20, 150, 160), -1)
        cv2.fillPoly(overlay, np.int32(Camera.make_poly(4) *
                     random.randint(12, 30) + coords[3]), (0, 0, 200))
        cv2.fillPoly(overlay, np.int32(Camera.make_poly(5) *
                     random.randint(12, 30) + coords[4]), (250, 25, 30))

        self.bg = bg
        self.overlay = overlay
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(camera_topic, Image, queue_size=1)

    def make_poly(sides):
        """Creates a polygon centered at (0, 0)"""
        phi = np.pi * 2 / sides
        angle = random.random() * 2 * np.pi
        rotations = [[[np.cos(i * phi + angle), -np.sin(i * phi + angle)], [np.sin(
            i * phi + angle), np.cos(i * phi + angle)]] for i in range(sides)]
        return np.array([[np.dot(m, (1, 0)) for m in rotations]], dtype=np.float64)

    def noise(image):
        row, col, ch = image.shape
        gauss = np.clip(np.random.randn(row, col, ch), -
                        0.9, 0.9).astype(np.uint8)
        gauss = gauss.reshape(row, col, ch)
        noisy = image + image * gauss
        noisy += np.array([random.randint(0, 4), 0, 0]).astype(np.uint8)
        noisy = np.clip(noisy, 0, 255)
        return noisy

    def publish(self, msg):
        self.pub.publish(msg)

    def publish_camera_image(self, y):
        cx = (self.w - self.tw)/2 + self.tw/2
        cy = (self.h - self.th / 2)/max_depth * y + self.th
        cropped_bg = self.bg[max(0, int(cy-self.th/2)):min(self.h + self.th, int(cy+self.th/2)),
                             max(0, int(cx-self.tw/2)):min(self.w, int(cx+self.tw/2))].copy()
        cropped_overlay = self.overlay[max(0, int(cy-self.th/2)):min(self.h + self.th, int(cy+self.th/2)),
                                       max(0, int(cx-self.tw/2)):min(self.w, int(cx+self.tw/2))]
        out = cv2.addWeighted(cropped_overlay, random.uniform(
            0.4, 0.5), cropped_bg, 0.6, 0, cropped_bg)
        out = Camera.noise(out)
        self.publish(self.bridge.cv2_to_imgmsg(out, encoding="bgr8"))


class Motion:
    """Represent the 1d motion of the vehicle"""

    def __init__(self, y=0, vy=0, drag_coeff=1.1, mass=1, fluid_density=997, gravity=9.81,
                 height=0.6, width=0.2, dt=0.05, env_depth=10, thrust_limit=2):

        self.drag_coeff = drag_coeff
        self.g = gravity
        self.fluid_density = fluid_density
        self.vehicle_dims = width, height
        self.mass = mass
        self.object_density = mass/(width * width * height)
        self.dt = dt
        self.y = y
        self.vy = vy
        self.env_constraints = np.array([-5, env_depth])
        self.thrust_limit = thrust_limit

    @staticmethod
    def constrain(val, lower, upper):
        return np.minimum(upper, np.maximum(lower, val))

    @staticmethod
    def isWithin(val, lower, upper):
        return lower <= val and val <= upper

    def volumeSubmerged(self):
        return (self.vehicle_dims[1] + min(self.y, 0)) * (self.vehicle_dims[0] ** 2)

    def update(self, vertical_thrust=0):
        thrust = self.constrain(
            vertical_thrust, -self.thrust_limit, self.thrust_limit)
        Fb = -self.fluid_density * self.g * self.volumeSubmerged()

        Fg = self.mass * self.g
        Fd = -0.5 * np.sign(self.vy) * self.drag_coeff * \
            self.vehicle_dims[0] ** 2 * self.vy ** 2
        F = Fb + Fg + Fd - thrust * self.mass + random.normalvariate(0, 0.1)
        newV = self.vy + (F/self.mass * self.dt)
        newY = self.y + newV * self.dt
        newY = self.constrain(
            newY, self.env_constraints[0], self.env_constraints[1])
        newV *= (newY < self.env_constraints[1]) * \
            (newY > self.env_constraints[0])
        self.y = newY
        self.vy = newV
        rospy.logdebug("Position: {} Velocity: {} Fb {} Fg {} Fd {} th {}".format(
            self.y, self.vy, Fb, Fg, Fd, thrust))

    def getPosition(self):
        return self.y


motion = Motion(
    y=0, vy=0, drag_coeff=drag_coefficient, mass=vehicle_mass, fluid_density=fluid_density,
    gravity=gravity, height=vehicle_height, width=vehicle_width, dt=time_interval, env_depth=max_depth,
    thrust_limit=10
)
camera = Camera()


def vertical_callback(msg):
    global thruster
    thruster = min(4, max(-4, msg.data))
    rospy.loginfo_throttle(5, "thrust: {0:.4f}".format(thruster))


def getY(depth):
    """Get the y position of a given depth in the screen"""
    return depth * display_scale + display_offset


def update_setpoint(depth):
    global setpoint
    rospy.loginfo_throttle_identical(
        5, "Received new setpoint: {0:.4f}".format(depth))
    setpoint = depth.new_setpoint
    return UpdateSetpointResponse(True)


count_within_threshold = 0

# Main
if __name__ == '__main__':
    # Initialise Game
    pygame.init()
    screen = pygame.display.set_mode(display_size)
    pygame.display.set_caption('Control Toy')

    # Setup ROS
    rospy.init_node("control_simulator", log_level=rospy.INFO)

    rospy.on_shutdown(pygame.quit)
    pub_depth = rospy.Publisher(depth_topic, Float64, queue_size=10)
    pub_setpoint = rospy.Publisher(setpoint_topic, Float64, queue_size=10)
    rospy.Subscriber(thruster_topic,
                     Float64, vertical_callback)

    sepoint_service = rospy.Service(
        setpoint_service_name, UpdateSetpoint, update_setpoint)
    running = True
    surface = pygame.display.get_surface()
    start_time = rospy.get_time()

    timer = rospy.Timer(rospy.Duration.from_sec(
        0.1), lambda e: camera.publish_camera_image(motion.getPosition()))

    # Primary loop
    while running and not rospy.is_shutdown():
        depth = motion.getPosition()

        # Publish telemetry
        pub_depth.publish(Float64(depth))
        pub_setpoint.publish(Float64(setpoint))

        # Draw
        reached_goal = (abs(depth - setpoint) < tolerance)
        setPointLineColour = (46, 204, 113) if reached_goal else (231, 76, 60)
        if not reached_goal:
            count_within_threshold = 0
        else:
            count_within_threshold += 1
        if count_within_threshold == 40:
            rospy.loginfo("reached depth={:.2f} in {:.2f} seconds".format(
                setpoint, rospy.get_time() - start_time))

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
        motion.update(thruster)

        # Handle user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                setpoint = Motion.constrain(
                    (pos[1] - display_offset) / display_scale, 0, max_depth)
                start_time = rospy.get_time()
                count = 0

        # Sleep
        rospy.sleep(time_interval)

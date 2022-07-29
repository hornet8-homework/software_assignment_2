#!/usr/bin/env python3
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
tolerance = 0.2
display_size = (250, 560)
max_depth = 15
drag_coefficient = 3
gravity = 9.81
fluid_density = 41.6666666667 * (gravity + 2) / gravity


camera_topic = "camera/image_raw"
depth_topic = "depth"
thruster_topic = "thruster"
setpoint_topic = "setpoint"
setpoint_service_name = "update_setpoint"


class Camera:
    def __init__(self, namespace):
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
        self.pub = rospy.Publisher(
            "/{}/{}".format(namespace, camera_topic), Image, queue_size=1)

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


class Vehicle:
    def __init__(self, name):
        self.name = name
        self.setpoint = 5
        self.thruster = 0
        self.vehicle_width = 0.2
        self.vehicle_height = 0.6
        self.vehicle_mass = 1
        self.motion = motion = Motion(
            y=0, vy=0, drag_coeff=drag_coefficient, mass=self.vehicle_mass, fluid_density=fluid_density,
            gravity=gravity, height=self.vehicle_height, width=self.vehicle_width, dt=time_interval, env_depth=max_depth,
            thrust_limit=10
        )
        self.camera = Camera(name)

        self.pub_depth = rospy.Publisher("/{}/{}".format(self.name, depth_topic), Float64, queue_size=10)
        self.pub_setpoint = rospy.Publisher(
            "/{}/{}".format(self.name, setpoint_topic), Float64, queue_size=10)
        rospy.Subscriber("/{}/{}".format(self.name, thruster_topic),
                         Float64, self.vertical_callback)
        self.setpoint_service = rospy.Service(
            "/{}/{}".format(self.name, setpoint_service_name), UpdateSetpoint, self.update_setpoint)

        rospy.Timer(rospy.Duration.from_sec(
            0.1), lambda e: self.camera.publish_camera_image(self.motion.getPosition()))

        self.count_within_threshold = 0

    def vertical_callback(self, msg):
        self.thruster = min(4, max(-4, msg.data))
        rospy.loginfo_throttle(
            5, "{} thrust: {}".format(self.name, self.thruster))

    def update(self):
        self.depth = self.motion.getPosition()
        # Publish telemetry
        self.pub_depth.publish(Float64(self.depth))
        self.pub_setpoint.publish(Float64(self.setpoint))
        if not self.reached_goal():
            self.count_within_threshold = 0
        else:
            self.count_within_threshold += 1
        if self.count_within_threshold == 40:
            rospy.loginfo("reached depth={:.2f} in {:.2f} seconds".format(
                self.setpoint, rospy.get_time() - start_time))
        self.motion.update(self.thruster)

    def reached_goal(self):
        return (abs(self.depth - self.setpoint) < tolerance)

    def getY(self, depth):
        """Get the y position of a given depth in the screen"""
        return depth * display_scale + display_offset

    def update_setpoint(self, depth):
        self.setpoint = depth
        self.setpoint = depth.new_setpoint
        return UpdateSetpointResponse(True)

    def draw(self):
        pygame.draw.rect(screen, (230, 126, 34), (w / 2, self.getY(self.depth),
                                                  self.vehicle_width * display_scale, self.vehicle_height * display_scale), 0)
        pygame.draw.lines(screen, setPointLineColour, False,
                          ((0, self.getY(self.setpoint)), (w, self.getY(self.setpoint))), 3)


# Main
if __name__ == '__main__':

    # Setup ROS
    rospy.init_node("control_simulator", log_level=rospy.INFO)

    gui = rospy.get_param("~gui", True)
    if gui:
        try:
            import pygame
            # Initialise Game
            pygame.init()
            screen = pygame.display.set_mode(display_size)
            pygame.display.set_caption('Control Toy')
            rospy.on_shutdown(pygame.quit)
            surface = pygame.display.get_surface()
        except:
            rospy.logerr("Could not import pygame, GUI disabled")
            gui = False

    running = True
    start_time = rospy.get_time()

    vehicle = Vehicle(rospy.get_param("~vehicle", "rocket"))

    blue = (41, 128, 185)
    green = (46, 204, 113)
    red = (231, 76, 60)

    # Primary loop
    while running and not rospy.is_shutdown():
        vehicle.update()

        # Draw
        setPointLineColour = green if vehicle.reached_goal() else red
        if gui:
            screen.fill((0, 0, 0))
            w, h = surface.get_width(), surface.get_height()
            pygame.draw.rect(screen, blue, (0, vehicle.getY(0), w, vehicle.getY(
                max_depth + vehicle.vehicle_height) - display_offset), 0)  # water
            pygame.draw.rect(screen, (255, 255, 255),
                    (0, 0, w, vehicle.getY(0)), 0)  # water surface
            vehicle.draw()
            pygame.display.update()

            # Handle user input
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    vehicle.setpoint = Motion.constrain(
                        (pos[1] - display_offset) / display_scale, 0, max_depth)
                    start_time = rospy.get_time()
                    count = 0

        # Sleep
        rospy.sleep(time_interval)

#!/usr/bin/env python
# coding: utf-8
import rospy
import threading
import time
import sys
import tf.transformations as tftr
from numpy import *

from geometry_msgs.msg import Pose, Point, Vector3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from tello_driver.msg import TelloStatus


lock = threading.Lock()
file_obj = open("drone_data.txt", "w")


class Controller:

    def __init__(self):
        self.status = None
        self.odometry = None
        self.i = 0
        self.start_odom = None

        "ROS stuff"
        self.status_sub = rospy.Subscriber("/tello/status", TelloStatus, self.tello_status_callback)
        self.odom_sub = rospy.Subscriber("/tello/odom", Odometry, self.odometry_callback)
        self.imu_sub = rospy.Subscriber("/tello/imu", Imu, self.imu_callback)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        #self.emergency_pub = rospy.Publisher('/tello/emergency', Empty, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.way_points = [[1, 0, -1], [1, 1, -1], [0, 1, -1], [0, 0, -1]]
        self.max_velocity = 1
        self.position_error = 0
        self.first_point = True
        self.vel = [0, 0, 0]
        print("Start")
        
        self.dt = 0.03

        # trajectory parameters (initial)
        self.laps = 1   # quantity laps
        self.velocity = 0.1
        self.A_x, self.A_y = 1., 1.
        self.phi = -pi / 2.

        self.tf = 10  # time for lap. Just for c
        self.omega_x = 2 * pi / self.tf
        self.omega_y = 2 * self.omega_x     # frequencies for 8-like-trajectory
        self.N = self.tf / self.dt  # quantity of a trajectory points

    def start(self):
        attempt = 1
        while (self.status == None) or not self.status.is_flying:
            time.sleep(1)
            print("Trying to take off... (atempt: " + str(attempt) + ")")
            self.takeoff_pub.publish(Empty())
            attempt += 1
            if attempt > 10:
                print("Cannot take off. Please check battery power.")
                exit(0)

    def tello_status_callback(self, msg):
        lock.acquire()
        self.status = msg
        lock.release()

    def odometry_callback(self, msg):
        lock.acquire()
        self.odometry = msg
        lock.release()

    def imu_callback(self, msg):
        lock.acquire()
        self.imu = msg
        lock.release()

    def stop(self):
        time.sleep(1)
        self.land_pub.publish(Empty())
        time.sleep(5)
        self.status_sub.unregister()
        self.odom_sub.unregister()
        self.imu_sub.unregister()

    def setup(self, i):
        start_odom_xyz = self.start_odom.pose.pose.position
        xyz = self.odometry.pose.pose.position
        xyz.x -= start_odom_xyz.x
        xyz.y -= start_odom_xyz.y

        #self.start_position = [xyz.x, xyz.y, xyz.z]
        if i == 0:
            self.start_position = [0, 0, 0]
        else:
            self.start_position = self.way_points[i-1]
        self.desired_position = self.way_points[i]
        dist = sqrt((self.desired_position[0] - self.start_position[0])**2 +
                    (self.desired_position[1] - self.start_position[1])**2 +
                    (self.desired_position[2] - self.start_position[2])**2)
        self.tp = dist/self.max_velocity
        #print("tp", self.tp)
        self.t0 = time.time() - self.start_time

    def rotate(self, x, y, z, yaw, pitch, roll):
        Rx = mat([[1, 0, 0],
             [0, cos(roll), -sin(roll)],
             [0, sin(roll), cos(roll)]])
        Ry = mat([[cos(pitch), 0, sin(pitch)],
             [0, 1, 0],
             [-sin(pitch), 0, cos(pitch)]])
        Rz = mat([[cos(yaw), -sin(yaw), 0],
             [sin(yaw), cos(yaw), 0],
             [0, 0, 1]])
        R = Rz*Ry*Rx
        X = R*mat([[x], [y], [z]])
        return X

    def update(self, dt, t):
        t
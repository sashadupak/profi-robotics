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
        self.emergency_pub = rospy.Publisher('/tello/emergency', Empty, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        
        # way points parameters
        self.way_points = [[1, 0, -1], [1, 1, -1], [0, 1, -1], [0, 0, -1]]
        self.max_velocity = 1  # [m/s]
        self.desired_height = -2 # [m] (with minus)

        #self.dt = 0.03

        # initialize
        self.step = 0
        self.zero_t = 0
        self.f1 = 0

        # trajectory parameters (initial)
        self.laps = 1   # quantity laps
        self.velocity = 0.1
        self.A_x, self.A_y = 1., 1.
        self.phi = -pi / 2.

        # get traj length
        l = 0.  # trajectory length
        ti = 0.
        while ti <= self.tf:    # for one lap
            xi = self.A_x * cos(self.omega_x * ti + self.phi)
            yi = self.A_y * sin(self.omega_y * ti)
            dl = sqrt((xi - old_xi)**2 + (yi - old_yi)**2)
            old_xi, old_yi = xi, yi
            l += dl
            ti += self.dt

        self.tf = l / self.velocity  # time for lap. Just for c
        self.omega_x = 2 * pi / self.tf
        self.omega_y = 2 * self.omega_x     # frequencies for 8-like-trajectory
        #self.N = self.tf / self.dt  # quantity of a trajectory points

        dt = 0.03
        # initial conditions for orientation
        self.old_xi = self.A_x * cos(self.omega_x * dt * (self.tf / dt - 1) + self.phi)
        self.old_yi = self.A_y * sin(self.omega_y * dt * (self.tf / dt - 1))

        self.start_time = time.time()
        self.t0 = self.start_time
        print("Start")

    def start(self):
        attempt = 1
        while (self.status == None) or not self.status.is_flying:
            time.sleep(1)
            #print("Trying to take off... (atempt: " + str(attempt) + ")")
            print("Take off...")
            self.takeoff_pub.publish(Empty())
            attempt += 1
            if attempt > 5:
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
        print("Land")

    def setup(self):
        self.xyz_offset = self.odometry.pose.pose.position
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
        t1 = time.time() - self.start_time

        if self.odometry == None:
            return

        if t > 300:
            self.stop()
            exit(0)

        if keyboard.is_pressed('L') or keyboard.is_pressed('l'):
            self.stop()

        if keyboard.is_pressed('E') or keyboard.is_pressed('e'):
            self.emergency_pub.publish(Empty())

        # odom

        velocity = Twist()
        if self.step == 0: # set height
            xyz = self.odometry.pose.pose.position
            h_err = xyz.z - self.desired_height
            velocity.linear.x = 0
            velocity.linear.y = 0
            velocity.linear.z = -1.4*h_err
            if abs(h_err) < 0.15:
                if self.f1 == 0:
                    self.zero_t = time.time()
                    self.f1 = 1
                else:
                    if time.time() - self.zero_t > 1:
                        self.step = 1
        elif self.step == 1: # set position
            xyz = self.odometry.pose.pose.position
            q = self.odometry.pose.pose.orientation     # quaternion
            rpy = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))  # roll pitch yaw

            xyz.x -= self.xyz_offset.x
            xyz.y -= self.xyz_offset.y

            if t1 <= self.tf * self.laps:
                # compute trajectory point and orientation
                xi = self.A_x * cos(self.omega_x * t1 + self.phi)
                yi = self.A_y * sin(self.omega_y * t1)
                theta = arctan2(yi - self.old_yi, xi - self.old_xi)
                self.old_xi, self.old_yi = xi, yi

                # compute the derivative of trajectory point and orientation
                dot_xi = -self.A_x * self.omega_x * sin(self.omega_x * t1 + self.phi)
                dot_yi = self.A_y * self.omega_y * cos(self.omega_y * t1)

                velocity.linear.x = dot_xi #+ 0.1*(v[0] - X[0])
                velocity.linear.y = dot_yi #+ 0.1*(v[1] - X[1])
                velocity.angular.z = 0
            else:
                velocity.linear.x = 0.0
                velocity.linear.y = 0.0
                velocity.angular.z = 0.0
                print("Complete")
        self.cmd_vel_pub.publish(velocity)
        #test = str(self.t0) + ' ' + str(t1) + ' ' + str(self.tp)
        pos = str(xyz.x) + ' ' + str(xyz.y) + ' ' + str(xyz.z)
        #ang = str(rpy[0]) + ' ' + str(rpy[1]) + ' ' + str(rpy[2])
        #vels = str(X[0][0]) + ' ' + str(X[1][0]) + ' ' + str(X[2][0])
        #traj = str(v[0]) + ' ' + str(v[1]) + ' ' + str(v[2])
        traj = str(xi) + ' ' + str(yi) + ' ' + str(desired_height)
        file_obj.write(pos + ' ' + traj + ' ' + str(t1) + '\n \n')


if __name__ == '__main__':
    rospy.init_node('drone_control_node')
    controller = None

    try:
        controller = Controller()

        time.sleep(1)
        controller.start()
        time.sleep(3)
        controller.setup()

        start = rospy.get_time()
        previous = rospy.get_time()

        while not rospy.is_shutdown():
            time.sleep(0.01)

            t = rospy.get_time() - start

            controller.update(t - previous, t)
            previous = t

        #except KeyboardInterrupt:
    finally:
        #time.sleep(1)
        controller.stop()
        self.status_sub.unregister()
        self.odom_sub.unregister()
        self.imu_sub.unregister()
        file_obj.close()
        del controller

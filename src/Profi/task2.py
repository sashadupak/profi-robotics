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


class Controller:

    def __init__(self):
        self.status = None
        self.odometry = None
        self.imu = None
        self.state = 0
        self.start_odom = None

        self.h = float(input("h: "))
        self.theta = float(input("theta: "))
        self.dh = float(input("dh: "))

        "ROS stuff"
        self.status_sub = rospy.Subscriber("/tello/status", TelloStatus, self.tello_status_callback)
        self.odom_sub = rospy.Subscriber("/tello/odom", Odometry, self.odometry_callback)
        self.imu_sub = rospy.Subscriber("/tello/imu", Imu, self.imu_callback)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

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

        # self.robotino_cmd_vel_pub.unregister()
        self.status_sub.unregister()
        self.odom_sub.unregister()
        self.imu_sub.unregister()

    def update(self, dt, t):
        if self.odometry == None or self.imu == None or self.status == None:
            return

        if self.start_odom == None:
            self.start_odom = self.odometry

        start_odom_xyz = self.start_odom.pose.pose.position
        start_odom_q = self.start_odom.pose.pose.orientation     # quaternion
        start_odom_rpy = tftr.euler_from_quaternion((start_odom_q.x, start_odom_q.y, start_odom_q.z, start_odom_q.w))  # roll pitch yaw

        odom_xyz = self.odometry.pose.pose.position
        odom_q = self.odometry.pose.pose.orientation     # quaternion
        odom_rpy = tftr.euler_from_quaternion((odom_q.x, odom_q.y, odom_q.z, odom_q.w))  # roll pitch yaw

        odom_xyz.x -= start_odom_xyz.x
        odom_xyz.y -= start_odom_xyz.y
        odom_rpy = (odom_rpy[0] - start_odom_rpy[0], odom_rpy[1] - start_odom_rpy[1], odom_rpy[2] - start_odom_rpy[2])

        # print(odom_xyz, odom_rpy, imu_rpy)

        e_z = 0
        e_theta = 0
        if self.state == 0:
            e_z = self.h + odom_xyz.z
            print("1. " + str(e_z))

            if abs(e_z) <= 0.2:
                self.state += 1
                self.state_start = t
        elif self.state == 1: 
            e_z = self.h + odom_xyz.z
            print("1. wait " + str(t - self.state_start))

            if t - self.state_start > 10:
                self.state += 1
        elif self.state == 2:
            e_z = self.h + odom_xyz.z
            e_theta = self.theta - odom_rpy[2]
            print("2. " + str(e_theta))

            if abs(e_theta) <= 0.2:
                self.state += 1
                self.state_start = t
        elif self.state == 3:
            e_z = self.h + odom_xyz.z
            e_theta = self.theta - odom_rpy[2]
            print("2. wait " + str(t - self.state_start))

            if t - self.state_start > 10:
                self.state += 1
        elif self.state == 4:
            e_z = self.h + self.dh + odom_xyz.z
            e_theta = self.theta - odom_rpy[2]
            print("3. " + str(e_z))

            if abs(e_z) <= 0.2:
                self.state += 1
                self.state_start = t
        elif self.state == 5: 
            e_z = self.h + self.dh + odom_xyz.z
            e_theta = self.theta - odom_rpy[2]
            print("3. wait " + str(t - self.state_start))

            if t - self.state_start > 10:
                self.state += 1

        velocity = Twist()
        velocity.linear.z = max(min(-e_z, 1.0), -1.0)
        velocity.angular.z = max(min(e_theta, 1.0), -1.0)
        self.cmd_vel_pub.publish(velocity)

        #text out
        with open("output.txt", 'a') as f:
            f.write("{},{},{}\n".format(str(t), str(e_z), str(e_theta)))



if __name__ == '__main__':
    rospy.init_node('task1_node')
    controller = None

    try:
        controller = Controller()

        controller.start()
        #time.sleep(3)

        start = rospy.get_time()
        previous = rospy.get_time()
        controller.state_start = start
        while controller.state < 6 and not rospy.is_shutdown():
            time.sleep(0.02)

            t = rospy.get_time() - start

            controller.update(t - previous, t)
            previous = t

    finally:
        time.sleep(1)
        controller.stop()
        del controller

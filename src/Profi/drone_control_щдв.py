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

        "ROS stuff"
        self.status_sub = rospy.Subscriber("/tello/status", TelloStatus, self.tello_status_callback)
        self.odom_sub = rospy.Subscriber("/tello/odom", Odometry, self.odometry_callback)
        self.imu_sub = rospy.Subscriber("/tello/imu", Imu, self.imu_callback)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        #self.emergency_pub = rospy.Publisher('/tello/emergency', Empty, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.way_points = [[0, 0, 1], [1, 0, 1]]
        self.max_velocity = 0.2
        self.position_error = 0
        self.first_point = True
        file_obj = open("drone_data.txt", "w")
        print("Start")

    def start(self):
        self.takeoff_pub.publish(Empty())

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
        self.start_position = [xyz.x, xyz.y, xyz.z]
        self.desired_position = self.way_points[i]
        dist = sqrt((desired_position[0] - start_position[0])**2 +
                    (desired_position[1] - start_position[1])**2 +
                    (desired_position[2] - start_position[2])**2)
        self.tp = dist/self.max_velocity
        self.t0 = time.time()

    def update(self, dt, t):
        if self.odometry == None:
            return

        if t > 15:
            self.stop()

        xyz = self.odometry.pose.pose.position
        q = self.odometry.pose.pose.orientation     # quaternion
        rpy = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))  # roll pitch yaw

        #low-pass filter
        #kf = 0.7
        #lp = (1 - kf)*current_value + kf*lp_old
        #lp_old = lp
        #complementary filter
        #a = 0.5
        #cmp = (1 - a)*odom + a*imu

        velocity = Twist()
        
        if (abs(xyz.x) > 1.5) or (abs(xyz.y) > 1.5) or (abs(xyz.z) > 1.5):
            velocity.linear.x = 0.0
            velocity.linear.y = 0.0
            velocity.angular.z = 0.0
            print("Waring! Drone position is out of bounds. Stable now, type 'L' to land.")
            cmd = input()
            if cmd == 'L' or cmd == 'l':
                controller.stop()
        else:
            #t = time.time() - self.start_time
            """
            self.position_error = 
            if self.position_error < 0.05:
                start_position = [xyz.x, xyz.y, xyz.z]
                desired_position = way_points[i]
                dist = sqrt((desired_position[0] - start_position[0])**2 +
                            (desired_position[1] - start_position[1])**2 +
                            (desired_position[2] - start_position[2])**2)
                tp = dist/self.max_velocity
                t0 = time.time()
            """
            if (self.first_point == True):
                Controller.setup(self.i)
                self.first_point = False
            if (time.time() - self.t0 - self.start_time < self.tp):
                velocity = []
                for j in range(3):
                    velocity.append((math.pi*(self.desired_position[j] - self.start_position[j])*math.sin((math.pi*(t - self.t0))/self.tp))/(2*self.tp))
                velocity.linear.x = velocity[0]
                velocity.linear.y = velocity[1]
                velocity.angular.z = velocity[2]
            else:
                self.i += 1
                if i <= len(self.way_point()):
                    Controller.setup(self.i)
                else:
                    Controller.stop()
                velocity.linear.x = 0.0
                velocity.linear.y = 0.0
                velocity.angular.z = 0.0
                print("Complete")
        self.cmd_vel_pub.publish(velocity)
        pos = str(xyz.x) + ' ' + str(xyz.y) + ' ' + str(xyz.z)
        ang = str(rpy.x) + ' ' + str(rpy.y) + ' ' + str(rpy.z)
        file_obj.write(pos + ' ' + ang + ' ' + str(t) + '\n \n')


if __name__ == '__main__':
    rospy.init_node('drone_control_node')
    controller = None

    try:
        controller = Controller()

        time.sleep(1)
        controller.start()

        start = rospy.get_time()
        previous = rospy.get_time()
        start_time = time.time()
        t0 = start_time
        while not rospy.is_shutdown():
            time.sleep(0.02)

            t = rospy.get_time() - start

            controller.update(t - previous, t)
            previous = t

            #if (t > 10):
            #    break
    finally:
        time.sleep(1)
        controller.stop()
        del controller

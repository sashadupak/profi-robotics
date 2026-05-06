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
        self.xyz_offset = None

        "ROS stuff"
        self.status_sub = rospy.Subscriber("/tello/status", TelloStatus, self.tello_status_callback)
        self.odom_sub = rospy.Subscriber("/tello/odom", Odometry, self.odometry_callback)
        self.imu_sub = rospy.Subscriber("/tello/imu", Imu, self.imu_callback)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        #self.emergency_pub = rospy.Publisher('/tello/emergency', Empty, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        
        self.way_points = [[1, -1.5, -1, radians(45)]] #format: [x, y, z, theta]
        self.max_velocity = 1
        #self.position_error = 0
        self.first_point = True
        #self.vel = [0, 0, 0]
        self.prog_stage = 0
        #self.x_lp_old = 0
        #self.y_lp_old = 0
        #self.z_lp_old = -1
        #self.odometry_offset = [0, 0, 0]
        self.x_offset = 0
        self.y_offset = 0
        print("Start")

    def start(self):
        attempt = 1
        while (self.status == None) or not self.status.is_flying:
            time.sleep(1)
            print("Trying to take off... (atempt: " + str(attempt) + ")")
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
        self.status_sub.unregister()
        self.odom_sub.unregister()
        self.imu_sub.unregister()

    def setup(self, i):
        #position
        #if i == 0:
        self.start_position = [0, 0, self.desired_height]
        #else:
        #    self.start_position = self.way_points[i-1]
        #self.desired_position = self.way_points[i]
        dist = sqrt((self.desired_position[0] - self.start_position[0])**2 +
                    (self.desired_position[1] - self.start_position[1])**2 +
                    (self.desired_height - self.start_position[2])**2)
        self.tp = dist/self.max_velocity
        #print("tp", self.tp)
        self.t0 = time.time() - self.start_time

        #orientation
        q = self.odometry.pose.pose.orientation     # quaternion
        rpy = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))  # roll pitch yaw
        self.start_angle = rpy[2]

    def odometry_offset():
        self.x_offset = self.odometry.pose.pose.position.x
        self.y_offset = self.odometry.pose.pose.position.y
        
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

        if self.start_odom == None:
            self.start_odom = self.odometry

        if t > 30:
            self.stop()
            exit(0)

        xyz.x = self.odometry.pose.pose.position.x - self.x_offset()
        xyz.y = self.odometry.pose.pose.position.y - self.y_offset()
        xyz.z = self.odometry.pose.pose.position.z
        q = self.odometry.pose.pose.orientation     # quaternion
        rpy = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))  # roll pitch yaw

        #xyz.x -= start_odom_xyz.x
        #xyz.y -= start_odom_xyz.y
        #acc = [self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, self.imu.linear_acceleration.z]
        #X = self.rotate(acc[0], acc[1], acc[2], rpy[2], rpy[1], rpy[0])
        #for j in range(3):
        #    self.vel[j] += X[j]*dt

        #low-pass filter
        #x_lp = (1 - 0.7)*xyz.x + 0.7*self.x_lp_old
        #self.x_lp_old = x_lp
        #y_lp = (1 - 0.7)*xyz.y + 0.7*self.y_lp_old
        #self.y_lp_old = y_lp
        #z_lp = (1 - 0.3)*xyz.z + 0.3*self.z_lp_old
        #self.z_lp_old = z_lp
            
        velocity = Twist()
        
        if self.prog_stage == 0: #set height
            self.setup(i)
            if (time.time() - self.t0 - self.start_time < self.tp):# or (abs(self.desired_height - xyz.z) > 0.15):
                desired_position_z = ((self.desired_position[2] - self.start_position[2])/2)*(1 - cos(pi*(t1 - self.t0)/self.tp)) + self.start_position[2]
                desired_velocity_z = (pi*(self.desired_position[2] - self.start_position[2])*sin((pi*(t1 - self.t0))/self.tp))/(2*self.tp)
                velocity.linear.x = 0.0
                velocity.linear.y = 0.0
                velocity.linear.z = desired_velocity_z + 0.3*(desired_position_z - xyz.z)
            else:
                velocity.linear.z = 0
                time.sleep(1)
                self.prog_stage = 1
            
        elif self.prog_stage == 1: #set position and orientation
            self.odometry_offset()
            self.setup(self.i)
            if (self.first_point == True):
                self.setup(self.i)
                self.first_point = False
            if (time.time() - self.t0 - self.start_time < self.tp):# or (abs(self.desired_x - xyz.x) > 0.15) or (abs(self.desired_y - xyz.y) > 0.15):
                v = []
                s = []
                for j in range(3):
                    v.append((pi*(self.desired_position[j] - self.start_position[j])*sin((pi*(t1 - self.t0))/self.tp))/(2*self.tp))
                    s.append(((self.desired_position[j] - self.start_position[j])/2)*(1 - cos(pi*(t1 - self.t0)/self.tp)) + self.start_position[j])
                    
                    dot_ang = (pi*(self.desired_angle - self.start_angle)*sin((pi*(t1 - self.t0))/self.tp))/(2*self.tp)
                    ang = ((self.desired_angle - self.start_angle)/2)*(1 - cos(pi*(t1 - self.t0)/self.tp)) + self.start_angle
                    V = self.rotate(v[0], v[1], v[2], rpy[2], rpy[1], rpy[0])
                    S = self.rotate(s[0], s[1], s[2], rpy[2], rpy[1], rpy[0])
                velocity.linear.x = V[0] + 0.3*(S[0] - xyz.x)
                velocity.linear.y = V[1] + 0.3*(S[1] - xyz.y)
                velocity.angular.z = dot_ang + 0.3*(ang - rpy[2])
                
            else:
                self.i += 1
                if self.i < len(self.way_points):
                    self.setup(self.i)
                else:
                    self.stop()
                velocity.linear.x = 0.0
                velocity.linear.y = 0.0
                velocity.angular.z = 0.0

        self.cmd_vel_pub.publish(velocity)
            
        """
        if (abs(xyz.x) > 1.5) or (abs(xyz.y) > 1.5) or (abs(xyz.z) > 2.5):
            velocity.linear.x = 0.0
            velocity.linear.y = 0.0
            velocity.angular.z = 0.0
            print("Warning! Drone position is out of bounds. Stable now, type 'L' to land.")
            self.cmd_vel_pub.publish(velocity)
            print(xyz, self.start_odom.pose.pose.position)
            cmd = raw_input()
            if cmd == 'L' or cmd == 'l':
                controller.stop()
                exit(0)
        else:
        """
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
                
        test = str(self.t0) + ' ' + str(t1) + ' ' + str(self.tp)
        pos = str(xyz.x) + ' ' + str(xyz.y) + ' ' + str(xyz.z)
        ang = str(rpy[0]) + ' ' + str(rpy[1]) + ' ' + str(rpy[2])
        vels = str(X[0][0]) + ' ' + str(X[1][0]) + ' ' + str(X[2][0])
        #traj = str(v[0]) + ' ' + str(v[1]) + ' ' + str(v[2])
        file_obj.write(vels + ' ' + pos + ' ' + ang + ' ' + str(t) + '\n \n')


if __name__ == '__main__':
    rospy.init_node('drone_control_node')
    controller = None

    try:
        controller = Controller()

        print("Initial data")
        controller.desired_height = -float(input("height [m]: "))
        controller.desired_position = []
        controller.desired_position.append(float(input("x [m]: ")))
        controller.desired_position.append(float(input("y [m]: ")))
        controller.desired_angle = float(input("theta angle [deg]: "))

        time.sleep(1)
        controller.start()
        time.sleep(3)

        start = rospy.get_time()
        previous = rospy.get_time()
        controller.start_time = time.time()
        controller.t0 = controller.start_time
        while not rospy.is_shutdown():
            time.sleep(0.02)
            t = rospy.get_time() - start
            controller.update(t - previous, t)
            previous = t

    finally:
        time.sleep(1)
        controller.stop()
        del controller

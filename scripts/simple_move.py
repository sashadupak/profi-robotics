#!/usr/bin/env python

from __future__ import division
import time
from math import sin, cos, sqrt, pi, atan2, tan, asin, copysign, atan

import rospy
from geometry_msgs.msg import Twist, Pose, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from hector_uav_msgs.srv import EnableMotors

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tf.transformations as tftr


lower = np.array([0,0,0])
upper = np.array([100,100,100])

focus = 160
k_lp = 0.75
lp_next = [320/2, 240/2]
lp_closest = [320/2, 240/2]

kv = 1.05
restrict_turbo = False


class SimpleMover():

    def __init__(self):
        rospy.init_node('simple_mover', anonymous=True)

        if rospy.has_param('/profi2021_bachelor_solution/altitude_desired'):
            self.altitude_desired = rospy.get_param('/profi2021_bachelor_solution/altitude_desired')
        else:
            rospy.logerr("Failed to get param '/profi2021_bachelor_solution/altitude_desired'")

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("cam_1/camera/image", Image, self.camera_cb)
        rospy.Subscriber("/ground_truth/state", Odometry, self.odometry_clb)
        rospy.Subscriber("/ground_truth_to_tf/euler", Vector3Stamped, self.orientation_clb)
        self.rate = rospy.Rate(10)

        self.cv_bridge = CvBridge()

        rospy.on_shutdown(self.shutdown)

        self.position = None
        self.direction = pi
        self.vx = 0
        self.vy = 0
        self.ex = 0
        self.ey = 0


    def camera_cb(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        if self.position is not None:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)

            image = self.stabilize_image(cv_image, self.position.z, -self.pitch, -self.roll, self.yaw)
            mask = self.stabilize_image(mask, self.position.z, -self.pitch, -self.roll, self.yaw)
            image = self.detect_line(image, mask)


    def odometry_clb(self, msg):
        self.position = msg.pose.pose.position
        self.velocity = msg.twist.twist.linear


    def orientation_clb(self, msg):
        self.roll = msg.vector.x
        self.pitch = msg.vector.y
        self.yaw = msg.vector.z


    def show_image(self, img):
        cv2.imshow("Camera 1 from Robot", img)
        cv2.waitKey(1)


    def enable_motors(self):
        try:
            rospy.wait_for_service('enable_motors', 2)
            call_service = rospy.ServiceProxy('enable_motors', EnableMotors)
            response = call_service(True)
        except Exception as e:
            print("Error while try to enable motors: ")
            print(e)


    def stabilize_image(self, image, height, roll, pitch, yaw):
        h, w = image.shape[:2]
        fx = focus
        fy = focus
        cx = w/2
        cy = h/2

        Rx = np.matrix([[1, 0, 0, 0],
                        [0, cos(roll), -sin(roll), 0],
                        [0, sin(roll), cos(roll), 0],
                        [0, 0, 0, 1]])
        Ry = np.matrix([[cos(pitch), 0, sin(pitch), 0],
                        [0, 1, 0, 0],
                        [-sin(pitch), 0, cos(pitch), 0],
                        [0, 0, 0, 1]])
        R = Rx*Ry
        
        A1 = np.matrix([[1/fx, 0, -cx/fx],
                        [0, 1/fy, -cy/fy],
                        [0, 0, 0],
                        [0, 0, 1]])
        A2 = np.matrix([[fx, 0, cx, 0],
                        [0, fy, cy, 0],
                        [0, 0, 1, 0]])
        dx = 1*tan(pitch)
        dy = 1*tan(-roll)
        dz = 1

        T = np.matrix([[1, 0, 0, dx],
                       [0, 1, 0, dy],
                       [0, 0, 1, dz],
                       [0, 0, 0, 1]])
        H = A2*(T*(R*A1))

        image = cv2.warpPerspective(image, H, (w, h))
        return image


    def find_trajectory_coeffs(self, t0, t1, q0, q1, v0, v1):
        a0 = 0
        j0 = 0

        a1 = 0
        j1 = 0

        T = t1 - t0
        h = q1 - q0

        a = q0
        b = v0
        c = a0/2
        d = j0/6
        e = ((210*h-T*((30*a0-15*a1)*T + (4*j0+j1)*T**2 + 120*v0 + 90*v1))/(6*T**4))
        f = ((-168*h+T*((20*a0-14*a1)*T+(2*j0+j1)*T**2+90*v0+78*v1))/(2*T**5))
        g = ((420*h-T*((45*a0-39*a1)*T+(4*j0+3*j1)*T**2+216*v0+204*v1))/(6*T**6))
        k = ((-120*h+T*((12*a0-12*a1)*T+(j0+j1)*T**2+60*v0+60*v1))/(6*T**7))
        return [a, b, c, d, e, f, g, k]


    def polynomial_trajectory(self, t, t0, coeffs):
        [a, b, c, d, e, f, g, k] = coeffs
        q = a + b*(t - t0) + c*(t - t0)**2 + d*(t - t0)**3 + e*(t - t0)**4 + f*(t - t0)**5 + g*(t - t0)**6 + k*(t - t0)**7
        q_dot = b + 2*c*(t - t0) + 3*d*(t - t0)**2 + 4*e*(t - t0)**3 + 5*f*(t - t0)**4 + 6*g*(t - t0)**5 + 7*k*(t - t0)**6
        return [q, q_dot]


    def distance(self, p1, p2):
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


    def closest_point(self, point, array):
        closest = None
        min_val = float('inf') 
        n = len(array)
        for i in range(n):
            if (self.distance(point, array[i]) < min_val):
                min_val = self.distance(point, array[i])
                closest = array[i]
        return closest


    def next_point(self, point, array, direction):
        closest = None
        min_val = float('inf') 
        min_lim = 100/3
        n = len(array)
        for i in range(n):
            angle_diff = abs(direction - self.angle(point, array[i]))
            if abs(angle_diff) > pi:
                angle_diff -= 2*pi*copysign(1, angle_diff)
            if (min_lim < self.distance(point, array[i]) < min_val) and (point != array[i]) and (abs(angle_diff) < float(pi/2)):
                min_val = self.distance(point, array[i])
                closest = array[i]
        return closest


    def angle(self, p1, p2):
        return atan2(p1[1]-p2[1], p1[0]-p2[0])


    def detect_line(self, img, mask):
        global lp_next
        global lp_closest
        global kv
        height, width = img.shape[:2]

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=int(self.position.z/3))
        mask = cv2.GaussianBlur(mask, (9, 9), 0)   

        dist = cv2.distanceTransform(mask, distanceType=cv2.DIST_L2, maskSize=3)

        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
        skeleton = cv2.morphologyEx(dist, cv2.MORPH_TOPHAT, kernel)

        skeleton = np.uint8(skeleton * 255)
        ret, skeleton = cv2.threshold(skeleton,0,255,0)

        _, contours, h = cv2.findContours(skeleton, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        self.vx = 0
        self.vy = 0
        if len(contours) > 0:
            cnt = max(contours, key=lambda x: cv2.arcLength(x, False))
            pts = cnt[:, 0, :].tolist()
            
            if len(pts) > 4:
                closest = self.closest_point((width/2, height/2), pts)
                if closest is not None:
                    lp_closest = [(1-k_lp)*closest[0] + k_lp*lp_closest[0], (1-k_lp)*closest[1] + k_lp*lp_closest[1]]
                    self.ex = (width/2 - lp_closest[0])*self.position.z/focus
                    self.ey = (height/2 - lp_closest[1])*self.position.z/focus
                    next_point = self.next_point(lp_closest, pts, self.direction)
                    if next_point is not None:
                        lp_next = [(1-k_lp)*next_point[0] + k_lp*lp_next[0], (1-k_lp)*next_point[1] + k_lp*lp_next[1]]
                        self.direction = self.angle(lp_closest, lp_next)
                        self.vx = cos(self.direction)
                        self.vy = sin(self.direction)
                else:
                    kv = 1.5
                    restrict_turbo = True
                    self.ex += 0.005
                    self.ey += 0.005
        return img


    def take_off(self, height):

        self.enable_motors()

        while self.position is None:
            continue

        start_time = time.time()
        takeoff_time = 1*height
        z0 = self.position.z
        z1 = z0 + height
        coeffs_z = self.find_trajectory_coeffs(0, takeoff_time, z0, z1, 0, 0)

        twist_msg = Twist()

        while (time.time() - start_time < takeoff_time) and (not rospy.is_shutdown()):
            z, vz = self.polynomial_trajectory(time.time() - start_time, 0, coeffs_z)
            twist_msg.linear.z = 0.2*(z - self.position.z)
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()


    def spin(self):
        global kv

        self.take_off(self.altitude_desired)

        eps_x = 0
        eps_y = 0
        fast_count = 0
        int_ex = 0
        int_ey = 0

        last_time = time.time()
        twist_msg = Twist()
        course = self.direction
        course_count = 0

        while not rospy.is_shutdown():
            twist_msg.linear.z = 0.2*(self.altitude_desired - self.position.z)

            dt = time.time() - last_time
            last_time = time.time()
            deps_x = 2.7*(self.ex - eps_x)
            ux = 0.4*(1.0*deps_x + eps_x)
            eps_x += deps_x*dt
            deps_y = 2.7*(self.ey - eps_y)
            uy = 0.4*(1.0*deps_y + eps_y)
            eps_y += deps_y*dt

            
            # TURBO MODE
            e = sqrt(self.ex**2 + self.ey**2)
            err = course - self.direction
            if abs(err) > pi:
                err -= 2*pi*copysign(1, err)
            if (abs(err) < 5*pi/180) and (e < 0.4):
                course_count += 1
                if course_count > 2500:
                    if restrict_turbo:
                        kv = 2.5
                    else:
                        kv = 4.5
            else:
                course_count = 0
                course = self.direction

            twist_msg.linear.x = kv*self.vy + uy 
            twist_msg.linear.y = kv*self.vx + ux 

            self.cmd_vel_pub.publish(twist_msg)


    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


simple_mover = SimpleMover()
simple_mover.spin()

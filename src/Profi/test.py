import time
from math import *
import numpy as np
import cv2
from operator import itemgetter


def set_drone_position(point):
    Qpos0
    dist = sqrt((point[0] - Qpos[0])**2 + (point[1] - Qpos[1])**2 + (point[2] - Qpos[2])**2)
    tp = dist/max_vel
    t0 = time.time()
    while (time.time() - t0 < tp):
        t = time.time() - t0
        for j in range(3):
            Qpos[j] = ((point[j] - Qpos0[j])/2)*(1 - cos(pi*t/tp)) + Qpos0[j]
        err = vrep.simxSetObjectPosition(
            clientID, QuadricopterT, -1, (Qpos[0], Qpos[1], Qpos[2]), vrep.simx_opmode_oneshot)
        time.sleep(0.02)
    time.sleep(1)


#low-pass filter:
lp(i) = (1 - kf)*in(i) + kf*lp(i-1);

#high-pass filter
hp(i) = (1 - kf)*hp(i-1) + (1 - kf)*(in(i) - in(i-1))

#complementary filter
cmp[i] = (1 - a)*fir + a*sec



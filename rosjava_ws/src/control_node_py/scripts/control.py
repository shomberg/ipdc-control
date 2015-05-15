#!/usr/bin/env python

import sys
import rospy
from kuka_msgs.srv import *
from GetShapes.srv import *
from math import cos, sin, pi

def move_ptp_client(x, y, z):
    rospy.wait_for_service('move_ptp')
    try:
        move_ptp = rospy.ServiceProxy('move_ptp', MovePTP)
        resp1 = move_ptp(x, y, z, 0, 180, 0)
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def get_shapes_client():
    rospy.wait_for_service('get_shapes')
    try:
        get_shapes = rospy.ServiceProxy('get_shapes', GetShapes)
        resp1 = get_shapes(True)
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

n = 5
points = [(500, 200*sin(pi*2*i/n), 700 + 200*cos(pi*2*i/n)) for i in range(n+1)]

for pt in points:
    print pt
    print move_ptp_client(*pt)

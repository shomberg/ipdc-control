#!/usr/bin/env python

import sys
import rospy
from kuka_msgs.srv import *
from ipdc.srv import *
from interface_wrappers.srv import *
from math import cos, sin, pi

def pickup_client(x, y, grip):
    rospy.wait_for_service('pickup')
    try:
        pickup = rospy.ServiceProxy('pickup', PickupAt)
        resp1 = pickup(x,y,grip)
        print resp1
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def get_shapes_client():
    rospy.wait_for_service('get_shapes')
    try:
        get_shapes = rospy.ServiceProxy('get_shapes', GetShapes)
        resp1 = get_shapes(False)
        return (resp1.center_x, resp1.center_y)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#n = 5
#points = [(500, 200*sin(pi*2*i/n), 700 + 200*cos(pi*2*i/n)) for i in range(n+1)]

#for pt in points:
#    print pt
#    print move_ptp_client(*pt)
a = -.0008
b = -2.31
c = -2.43
d = .08
e = -85.98
f = -421.24

if __name__ == "__main__":
    pickup_client(400,200,False)
    (origX,origY) = get_shapes_client()
    transformX = []
    transformY = []
    for i in range(len(origX)):
        transformX.append(a*(origX[i]+e) + b*(origY[i]+f))
        transformY.append(c*(origX[i]+e) + d*(origY[i] + f))
    print transformX
    print transformY
    for i in range(len(transformX)):
        pickup_client(transformX[i],transformY[i],True)
        pickup_client(transformX[i]-200,transformY[i]+150,False)

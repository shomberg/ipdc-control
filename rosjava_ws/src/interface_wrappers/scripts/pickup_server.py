#!/usr/bin/env python

import sys
import rospy
from kuka_msgs.srv import *
from interface_wrappers.srv import *
from math import cos, sin, pi
from time import sleep


def move_ptp_client(x, y, z):
    rospy.wait_for_service('move_ptp')
    try:
        move_ptp = rospy.ServiceProxy('move_ptp', MovePTP)
        resp1 = move_ptp(x, y, z, 0, 180, 0)
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def set_grip_client(grip):
    rospy.wait_for_service('set_grip')
    try:
        set_grip = rospy.ServiceProxy('set_grip', SetGrip)
        resp1 = set_grip(grip)
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def handle_pickup(req):
    if not move_ptp_client(req.X, req.Y, 300):
        print "failed move to above"
        return PickupAtResponse(False)
    if not move_ptp_client(req.X, req.Y, 40):
        print "failed move to position"
        return PickupAtResponse(False)
    sleep(.5)
    if not set_grip_client(req.grip):
        print "failed grip"
        return PickupAtResponse(False)
    sleep(.5)
    if not move_ptp_client(req.X,req.Y,300):
        print "failed return to position"
        return PickupAtResponse(False)
    return PickupAtResponse(True)

def pickup_server():
    rospy.init_node('pickup_server')
    s = rospy.Service('pickup', PickupAt, handle_pickup)
    print "Ready to pick up."
    rospy.spin()


if __name__ == "__main__":
    pickup_server()

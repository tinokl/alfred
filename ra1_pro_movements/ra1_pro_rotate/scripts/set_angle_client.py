#!/usr/bin/env python

import rospy
import math
from ra1_pro_msgs.srv import *

def set_new_angle(joint_name, delta, duration):
    print "Waiting for Service"
    rospy.wait_for_service('rotate_angle')
    try:
        set_angle = rospy.ServiceProxy('rotate_angle', RotateAngle)
        resp1 = set_angle(joint_name, delta, duration)

        return resp1.error
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == "__main__":

    rospy.init_node('new_angle_generator', anonymous=True)

    new_angle = math.pi/10.0
    joint_name = 'servo_6'
    duration = 1.0

    print "New Angle: %s" % (new_angle)

    print "Set %s: ERRORCODE: %s" % (joint_name, set_new_angle(joint_name, new_angle, duration))




#!/usr/bin/env python
#test client for joint_states_listener

import roslib
import sys
import rospy
from g3_joint_states_listener_msgs.srv import *
import time
import sys

def call_return_joint_states(joint_name):
    rospy.wait_for_service("robot/joint_states_filtered")
    try:
        s = rospy.ServiceProxy("robot/joint_states_filtered", returnJointStates)
        resp = s(joint_name)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    return (resp.position)


#print out the positions, velocities, and efforts of the right arm joints
if __name__ == "__main__":
    joint_name = "left_w2"

    while(1):
        position = call_return_joint_states(joint_name)
        print "position:", position

        time.sleep(1)

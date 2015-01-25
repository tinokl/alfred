#!/usr/bin/env python

import sys
import rospy
from ra1_pro_msgs.srv import *
from geometry_msgs.msg import *

def set_new_cartesian_pose(waypoint):
    print "Waiting for Service"
    rospy.wait_for_service('move_cartesian')
    try:
        set_move_cartesian = rospy.ServiceProxy('move_cartesian', MoveCartesian)
        resp1 = set_move_cartesian(waypoint)

        return resp1.error
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == "__main__":

    rospy.init_node('new_move_cartesian', anonymous=True)

    new_pose = PoseStamped()
    new_pose.header.seq = 1
    new_pose.header.stamp = rospy.Time.now()
    new_pose.header.frame_id = "/base"
    new_pose.pose.orientation.x = 0.0
    new_pose.pose.orientation.y = 0.0
    new_pose.pose.orientation.z = 0.0
    new_pose.pose.orientation.w = 0.0
    new_pose.pose.position.x = 0.0
    new_pose.pose.position.y = 0.0
    new_pose.pose.position.z = -0.05

    #print "New Pose: %s" % (new_pose)

    #publish_new_pose_cartesian(new_pose)
    print "Set arm Cartesian : ERRORCODE: %s" % (set_new_cartesian_pose(new_pose))





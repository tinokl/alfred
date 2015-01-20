#!/usr/bin/env python

import sys
import rospy
from ra1_pro_msgs.srv import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker

def set_new_pose(pose):
    print "Waiting for Service"
    rospy.wait_for_service('move_pose')
    try:
        set_arm_pose = rospy.ServiceProxy('move_pose', MovePose)
        resp1 = set_arm_pose(pose)

        return resp1.error
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def publish_new_pose(pose):

    topic = 'visualization_marker'
    publisher = rospy.Publisher(topic, Marker)

    count = 0

    while count < 100:

        marker = Marker()
        marker.header.frame_id = "/base"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.x = pose.orientation.x
        marker.pose.orientation.y = pose.orientation.y
        marker.pose.orientation.z = pose.orientation.z
        marker.pose.orientation.w = pose.orientation.w
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z

        publisher.publish(marker)

        count += 1

        rospy.sleep(0.01)

if __name__ == "__main__":

    rospy.init_node('new_pose_generator', anonymous=True)

    new_pose = Pose()

    new_pose.position.x = 0.0
    new_pose.position.y = 2.79396772385e-09
    new_pose.position.z = 0.311586290598

    new_pose.orientation.x = 0.0
    new_pose.orientation.y = 0.0
    new_pose.orientation.z = 0.707106769085
    new_pose.orientation.w = 0.707106769085


    print "New Pose: %s" % (new_pose)

    publish_new_pose(new_pose)

    print "Set Arm: ERRORCODE: %s" % (set_new_pose(new_pose))




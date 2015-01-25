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
    publisher = rospy.Publisher(topic, Marker, queue_size=10)

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


if __name__ == "__main__":

    rospy.init_node('new_pose_generator', anonymous=True)

    new_pose = Pose()
    home = True

    #new_pose.position.x = 0.063595
    #new_pose.position.y = -0.034242
    #new_pose.position.z = 0.25728

    #new_pose.orientation.x = 0.67145
    #new_pose.orientation.y = -0.045775
    #new_pose.orientation.z = 0.67357
    #new_pose.orientation.w = 0.30554

    if home is True:
        # home pose
        new_pose.position.x = 0.0
        new_pose.position.y = 0.0
        new_pose.position.z = 0.29

        new_pose.orientation.x = 0.418
        new_pose.orientation.y = -0.372
        new_pose.orientation.z = 0.55
        new_pose.orientation.w = 0.61
    else:
        # neutral "almost init" pose
        new_pose.position.z = 0.355
        new_pose.orientation.z = 0.70711
        new_pose.orientation.w = 0.70711

    print "New Pose: %s" % (new_pose)

    publish_new_pose(new_pose)

    print "Set Arm: ERRORCODE: %s" % (set_new_pose(new_pose))




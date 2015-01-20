#!/usr/bin/env python

import sys
import rospy
from g3_move_arms_msgs.srv import *
from geometry_msgs.msg import *
#from visualization_msgs.msg import Marker


def set_new_cartesian_pose(move_group, waypoint):
    print "Waiting for Service"
    rospy.wait_for_service('arm_pose_cartesian')
    try:
        set_arm_pose_cartesian = rospy.ServiceProxy('arm_pose_cartesian', armPoseCartesian)
        resp1 = set_arm_pose_cartesian(move_group, waypoint)

        return resp1.error
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


#def publish_new_pose_cartesian(pose):

    #topic = 'visualization_marker'
    #publisher = rospy.Publisher(topic, Marker)

    #count = 0

    #while count < 100:

        #marker = Marker()
        #marker.header.frame_id = "/world"
        #marker.type = marker.CUBE
        #marker.action = marker.ADD
        #marker.scale.x = 0.1
        #marker.scale.y = 0.1
        #marker.scale.z = 0.1
        #marker.color.a = 1.0
        #marker.color.r = 1.0
        #marker.color.g = 0.0
        #marker.color.b = 0.0
        #marker.pose.orientation.x = pose.orientation.x
        #marker.pose.orientation.y = pose.orientation.y
        #marker.pose.orientation.z = pose.orientation.z
        #marker.pose.orientation.w = pose.orientation.w
        #marker.pose.position.x = pose.position.x
        #marker.pose.position.y = pose.position.y
        #marker.pose.position.z = pose.position.z

        #publisher.publish(marker)

        #count += 1

        #rospy.sleep(0.01)

if __name__ == "__main__":

    rospy.init_node('new_pose_generator_cartesian', anonymous=True)

    new_pose = PoseStamped()
    new_pose.header.seq = 1
    new_pose.header.stamp = rospy.Time.now()
    new_pose.header.frame_id = "base"
    new_pose.pose.orientation.x = 0.0
    new_pose.pose.orientation.y = 0.0
    new_pose.pose.orientation.z = 0.0
    new_pose.pose.orientation.w = 0.0
    new_pose.pose.position.x = -0.05
    new_pose.pose.position.y = 0.008
    new_pose.pose.position.z = 0.0
    # move_group = armPoseCartesianRequest.ARM_LEFT
    move_group = armPoseCartesianRequest.ARM_RIGHT

    #print "New Pose: %s" % (new_pose)

    #publish_new_pose_cartesian(new_pose)
    print "Set %s Cartesian : ERRORCODE: %s" % (move_group, set_new_cartesian_pose(move_group, new_pose))





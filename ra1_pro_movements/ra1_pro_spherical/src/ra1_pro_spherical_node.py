#!/usr/bin/env python

import roslib; roslib.load_manifest('ra1_pro_spherical')
import rospy
import math
from ra1_pro_msgs.srv import *
from ra1_pro_msgs.msg import *


class Ra1ProSpherical:

    def __init__(self):
        rospy.loginfo(rospy.get_name() + ": Starting Node")

        self.basic_cmd_service = rospy.Service('follow_point', BasicCMD, self.handle_basic_cmd)
        rospy.Subscriber("/ra1_pro/delta_ptn", DeltaPoint, self.delta_point_cb)

        self.joints_horizontal = ['servo_6']
        self.joints_vertical = ['servo_5', 'servo_4', 'servo_3']

        self.is_follow_ptn = False
        self.last_msg = None

        self.thresh_y = 10
        self.thresh_x = 10

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_follow_ptn is True:
                #print "try to follow point"
                self.follow_point()
            rate.sleep()

    def handle_basic_cmd(self, req):
        resp = BasicCMDResponse()

        if req.cmd == req.STOP:
            if self.is_follow_ptn is True:
                rospy.loginfo(rospy.get_name() + ": Stop to follow point")
                self.is_follow_ptn = False
                resp = resp.SUCCESS
            else:
                rospy.logerr(rospy.get_name() + ": I was not following a point!")
                resp = resp.ERROR
        elif req.cmd == req.START:
            if self.is_follow_ptn is False:
                rospy.loginfo(rospy.get_name() + ": Start to follow point")
                self.is_follow_ptn = True
                resp = resp.SUCCESS
            else:
                rospy.logerr(rospy.get_name() + ": I was already following a point!")
                resp = resp.ERROR
        else:
            rospy.logerr(rospy.get_name() + ": Unknown Command!")
            resp = resp.ERROR

        return resp

    def delta_point_cb(self, msg):
        self.last_msg = msg

    def follow_point(self):
        if self.last_msg is None:
            rospy.logerr(rospy.get_name() + ": No msgs with delta points received!")
            return

        duration = 0.0
        if math.fabs(self.last_msg.delta_x) > self.thresh_x:
            joint_name = self.joints_horizontal[0]
            d_angle = (math.pi/180.0) * 15
            d_angle = math.copysign(d_angle, self.last_msg.delta_x)
            error = self.set_angle(joint_name, d_angle, duration)
            rospy.loginfo(rospy.get_name() + ": Correcting horizontal with joint: " + joint_name)

        if math.fabs(self.last_msg.delta_y) > self.thresh_y:
            joint_name = self.joints_vertical[0]
            d_angle = (math.pi/180.0) * 15
            d_angle = math.copysign(d_angle, self.last_msg.delta_y)
            error = self.set_angle(joint_name, d_angle, duration)
            rospy.loginfo(rospy.get_name() + ": Correcting vertical with joint: " + joint_name)

    def set_angle(self, joint_name, delta, duration):
        rospy.loginfo(rospy.get_name() + ": Waiting for Service")
        rospy.wait_for_service('rotate_angle')
        try:
            set_angle = rospy.ServiceProxy('rotate_angle', RotateAngle)
            resp1 = set_angle(joint_name, delta, duration)

            return resp1.error
        except rospy.ServiceException, e:
            rospy.loginfo(rospy.get_name() + ":Service call failed: %s" % e)


if __name__ == '__main__':
    rospy.init_node('ra1_pro_spherical', anonymous=False)
    try:
        Ra1ProSpherical()
    except rospy.ROSInterruptException:
        pass

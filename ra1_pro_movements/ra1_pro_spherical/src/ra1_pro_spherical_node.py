#!/usr/bin/env python

import roslib; roslib.load_manifest('ra1_pro_spherical')
import rospy
import math
import random

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from ra1_pro_spherical.cfg import RA1ProSphericalConfig as ConfigType

from std_msgs.msg import *
from ra1_pro_msgs.srv import *
from ra1_pro_msgs.msg import *
from sensor_msgs.msg import JointState

class Ra1ProSpherical:

    def __init__(self):
        rospy.loginfo(rospy.get_name() + ": Starting Node")

        self.basic_cmd_service = rospy.Service('/alfred/follow_point', BasicCMD, self.handle_follow_sv)
        self.dance_sv = rospy.Service('/alfred/dance', BasicCMD, self.handle_dance_sv)

        rospy.Subscriber("/ra1_pro/delta_ptn", DeltaPoint, self.delta_point_cb)
        rospy.Subscriber("/joint_states", JointState, self.get_state_cb)
        self.joint_state = rospy.Publisher('/move_group/controller_joint_states', JointState, queue_size=10)

        #client = dynamic_reconfigure.client.Client("RA1ProSperical", timeout=30, config_callback=self.config_cb)
        self.server = DynamicReconfigureServer(ConfigType, self.config_cb)

        self.config = None

        self.joints_horizontal = ['servo_6']
        self.joints_vertical = ['servo_5', 'servo_4', 'servo_3']
        self.joints_dance = ['servo_2', 'servo_3', 'servo_4', 'servo_5', 'servo_6']

        self.is_dancing = False
        self.is_follow_ptn = False
        self.last_state_msg = None
        self.last_dp_msg = None

        self.state_arm = JointState()
        self.header = Header()
        self.header.frame_id = '/base'
        self.state_arm.header = self.header

        self.dance_joint = None
        self.dance_alpha = 0
        self.dance_amplitude = 0.2
        self.dance_incr = 40

        self.epsilon_y = 50
        self.epsilon_x = 50

        self.srate = 10
        self.rate = rospy.Rate(self.srate)
        while not rospy.is_shutdown():
            if self.is_follow_ptn == True:
                self.follow_point()
            elif self.is_dancing == True:
                self.dance_move()
            self.rate.sleep()

    def config_cb(config):
        self.config = config
        rospy.loginfo(rospy.get_name() + ": I am in")
        #self.rate = rospy.Rate(config.rate)
        #self.dance_amplitude = config.dance_amplitude
        #self.dance_incr = config.dance_incr

    def handle_follow_sv(self, req):
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

    def handle_dance_sv(self, req):
        resp = BasicCMDResponse()

        if req.cmd == req.STOP:
            if self.is_dancing is True:
                rospy.loginfo(rospy.get_name() + ": Stop to dance")
                self.is_dancing = False
                resp = resp.SUCCESS
            else:
                rospy.logerr(rospy.get_name() + ": I was not dancing!")
                resp = resp.ERROR
        elif req.cmd == req.START:
            self.rate = rospy.Rate(15)
            if self.is_dancing is False:
                rospy.loginfo(rospy.get_name() + ": Start to dance!")
                self.is_dancing = True
                self.dance_joint = random.randint(0, len(self.joints_dance)-1)
                resp = resp.SUCCESS
            else:
                rospy.logerr(rospy.get_name() + ": I was already dancing!")
                resp = resp.ERROR
        else:
            rospy.logerr(rospy.get_name() + ": Unknown Command!")
            resp = resp.ERROR
        return resp

    def delta_point_cb(self, msg):
        self.last_dp_msg = msg

    def get_state_cb(self, msg):
        if msg.name[0] == 'servo_2':
            self.last_state_msg = msg

    def follow_point(self):
        if self.last_dp_msg is None:
            #rospy.logerr(rospy.get_name() + ": No msgs with delta points received!")
            return
        if self.last_state_msg is None:
            #rospy.logerr(rospy.get_name() + ": No state msgs received!")
            return
        #if abs(rospy.Time.now() - self.last_dp_msg.header.stamp) > 10:
        #    rospy.logerr(rospy.get_name() + ": No new msgs received!")
        #    return

        move_joints = []
        move_positions = []
        if math.fabs(self.last_dp_msg.delta_x) > self.epsilon_x:
            joint_name = self.joints_horizontal[0]
            index = self.last_state_msg.name.index(joint_name)
            real_angle = self.last_state_msg.position[index]

            d_angle = (math.pi/180.0) * self.last_dp_msg.delta_x/50
            d_angle = math.copysign(d_angle, self.last_dp_msg.delta_x) * (-1.0)
            new_angle = real_angle + d_angle

            move_joints.append(joint_name)
            move_positions.append(new_angle)
            # rospy.loginfo(rospy.get_name() + ": Correcting horizontal with joint: " + joint_name)

        if math.fabs(self.last_dp_msg.delta_y) > self.epsilon_y:
            pick = random.randint(0, len(self.joints_vertical)-1)
            joint_name = self.joints_vertical[pick]
            index = self.last_state_msg.name.index(joint_name)
            real_angle = self.last_state_msg.position[index]

            d_angle = (math.pi/180.0) * self.last_dp_msg.delta_y/50
            d_angle = math.copysign(d_angle, self.last_dp_msg.delta_y)
            new_angle = real_angle + d_angle

            move_joints.append(joint_name)
            move_positions.append(new_angle)
            # rospy.loginfo(rospy.get_name() + ": Correcting vertical with joint: " + joint_name)

        self.send_robot_state(move_joints, move_positions)

    def dance_move(self):
        move_joints = []
        move_positions = []
        joint_name = self.joints_dance[self.dance_joint]
        move_joints.append(joint_name)
        index = self.last_state_msg.name.index(joint_name)
        real_angle = self.last_state_msg.position[index]
        print config["rate"]
        d_angle = math.sin(self.dance_alpha*math.pi/180) * self.dance_amplitude
        self.dance_alpha += self.dance_incr % 360
        #if self.dance_alpha > 360:
        #    self.dance_alpha = 0

        rospy.loginfo(rospy.get_name() + ": Dance Alpha: " + str(self.dance_alpha))

        new_angle = real_angle + d_angle
        move_positions.append(new_angle)
        self.send_robot_state(move_joints, move_positions)

    def send_robot_state(self, joints, positions):
        self.header.stamp = rospy.Time.now()

        self.state_arm.name = joints
        self.state_arm.position = positions
        self.state_arm.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.state_arm.effort = []
        self.joint_state.publish(self.state_arm)


if __name__ == '__main__':
    rospy.init_node('ra1_pro_spherical', anonymous=False)
    try:
        Ra1ProSpherical()
    except rospy.ROSInterruptException:
        pass

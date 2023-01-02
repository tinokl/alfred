#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com
# RA1-Pro-AREXX Controller

# Advertises Service "ra1_pro_cmd" with BasicCMD messages
# Subscribes on topic "/ra1_pro/cmd" with Ra1ProSimpleMove messages
# Subscribes on topic "/move_group/controller_joint_states" with JointState messages
# Publishes connection information on topic "ra1_pro/feedback" with strings

## Ra1ProSimpleMove:
#Header header
#int8 servo
#float64 direction
#float64 position
#float64 percent

# Normal Usage:
# - Connect
# - Client sends "Ready"
# - Server sends "ON"
# - Server sends commands (Move)
# - ...
# - Server sends "OFF"

# S - Which servo 	(S1,S2,S3,S4,S5 or S6)
# N - The position 	(Min. N900 ..... P900 max)
# V - The speed		(Fast V0  ......  V10 slow)

import roslib; roslib.load_manifest('ra1_pro')
import rospy
import time
import math

from motor_driver import PCA9685

from std_msgs.msg import *
from sensor_msgs.msg import JointState
from ra1_pro_msgs.msg import *
from ra1_pro_msgs.srv import *


class Ra1Pro:

    def __init__(self):
        rospy.loginfo(rospy.get_name() + ": Starting Node")

        self.offline = rospy.get_param('~offline')
        self.ready = False
        self.connected = False

        self.driver_pub = rospy.Publisher('ra1_pro/feedback', String, queue_size=10)
        self.joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.basic_cmd_service = rospy.Service('ra1_pro_cmd', BasicCMD, self.handle_basic_cmd)

        rospy.Subscriber("/ra1_pro/cmd", Ra1ProSimpleMove, self.simple_movement)
        rospy.Subscriber("/ra1_pro/neural_link", Ra1ProNeuralLink, self.neural_link)
        rospy.Subscriber("/move_group/controller_joint_states", JointState, self.get_trajectory)

        rate = rospy.Rate(10)

        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        self.pwd_set = True

        self.msg_not_ready = ": System is not ready, what?"

        self.servos = 6

        # save servo positions
        #                      0    1    2    3    4    5
        self.servo_curr_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_new_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # less than these values
        self.servo_name = ["gripper", "wrist", "servo3", "arm", "shoulder", "base"]
        self.servo_pos_max = [1650, 2405, 2500, 2700, 2800, 2500]
        self.servo_pos_center = [1000, 1500, 1550, 1650, 1600, 1550]
        self.servo_pos_min = [950, 505, 430, 500, 700, 500]

        # 500 == 0 degree
        # 1000 == 45 degree
        # 1500 == 90 degree
        # 2000 == 135 degree
        # 2500 == 180 degree

        self.state_arm = JointState()
        self.state_gripper = JointState()
        self.header = Header()

        rospy.loginfo(rospy.get_name() + ": Waiting for start")

        while not rospy.is_shutdown():
            if self.pwd_set or self.offline:
                self.send_robot_state()
                if self.offline is False:
                    try:
                        raw, text = self.pwm.read(0x00)
                        string = rospy.get_name() + ": Connection response: %s" % text
                        #self.check_response(str(serial_read))
                        self.driver_pub.publish(String(string))
                    except:
                        rospy.logerr(rospy.get_name() + ": Connection error occured!")
            rate.sleep()

    def handle_basic_cmd(self, req):
        resp = BasicCMDResponse()

        if req.cmd == req.STOP:
            if self.ready is True:
                rospy.logerr(rospy.get_name() + self.msg_not_ready)
                #resp = resp.ERROR
            self.ready = False
            #self.cleanup()
            rospy.loginfo(rospy.get_name() + ": Closing")
            resp = resp.SUCCESS
        elif req.cmd == req.START:
            if self.ready is not True:
                if self.pwd_set or self.offline:
                    self.norm_position()
                    self.ready = True
                    rospy.loginfo(rospy.get_name() + ": Starting")
                    rospy.sleep(2)
                    self.home_position()
                    resp = resp.SUCCESS
            else:
                rospy.logerr(rospy.get_name() + self.msg_not_ready)
                resp = resp.ERROR
        return resp

    """
    def check_response(self, string):
        cmd_on = "Alfred # ON executed"
        cmd_off = "Alfred # OFF init"
        cmd_wait = "Alfred # wait ON"
        cmd_error = "Alfred # COMMAND Error got: ON0000000000000000000000000000000000000000000000"

        if string == cmd_on:
            self.connected = True
        if string == cmd_off:
            self.connected = False
            self.ready = False
        if string == cmd_wait:
            self.connected = False
            self.ready = False
        if string == cmd_error:
            self.connected = True
            self.ready = True
    """

    def simple_movement(self, data):
        if data.direction == 0 and self.ready is True:
            self.move_position(data)
        elif data.direction > 0 and self.ready is True:
            self.move_direction(data)
        else:
            rospy.logerr(rospy.get_name() + self.msg_not_ready)

    def neural_link(self, data):
        if self.ready is True:
            # normalize neural input within servor min max range
            self.servo_new_pos[0] = servo_pos_min[0] + (servo_pos_max[0]-servo_pos_min[0]) * data.servo_1_state
            self.servo_new_pos[1] = servo_pos_min[1] + (servo_pos_max[1]-servo_pos_min[1]) * data.servo_2_state
            self.servo_new_pos[2] = servo_pos_min[2] + (servo_pos_max[2]-servo_pos_min[2]) * data.servo_3_state
            self.servo_new_pos[3] = servo_pos_min[3] + (servo_pos_max[3]-servo_pos_min[3]) * data.servo_4_state
            self.servo_new_pos[4] = servo_pos_min[4] + (servo_pos_max[4]-servo_pos_min[4]) * data.servo_5_state
            self.servo_new_pos[5] = servo_pos_min[5] + (servo_pos_max[5]-servo_pos_min[5]) * data.servo_6_state
            self.send_move_command()

    def get_trajectory(self, data):
        return

        if self.ready is False:
            rospy.logerr(rospy.get_name() + self.msg_not_ready)
            return

        new_position = False
        servo_start = 0
        servo_end = len(data.name)

        for s in range(servo_start, servo_end):
            index = self.servo_name.index(data.name[s])
            norm_position = ((data.position[s]*180/math.pi)*10)
            pos_round = round(norm_position/10)*10

            # gripper movement must be extra scaled
            if index is 0:
                pos_round *= -100

            #speed = round(abs(data.velocity[s-1]*10))
            #if speed < 1:
            #    speed = 1
            #self.servo_speed[s] = speed

            if (abs(pos_round) <= self.servo_pos_max[index]) and (abs(pos_round) >= self.servo_pos_min[index]):
                if pos_round != self.servo_curr_pos[index]:
                    self.servo_new_pos[index] = pos_round
                    new_position = True
            else:
                error = ": ERROR position {0} servo {1} out of bounds - position must be smaller than {2}".format(round(norm_position), index, self.servo_pos_max[index])
                self.servo_new_pos[index] = math.copysign(self.servo_pos_max[index], pos_round)
                rospy.logerr(rospy.get_name() + error)

        if new_position:
            self.send_move_command()

    def send_robot_state(self):
        self.state_arm = JointState()
        self.state_gripper = JointState()

        self.header = Header()
        self.header.stamp = rospy.Time.now()
        self.header.frame_id = '/base'

        self.state_arm.header = self.header
        self.state_gripper.header = self.header

        self.state_arm.name = ['servo_2', 'servo_3', 'servo_4', 'servo_5', 'servo_6']
        self.state_arm.position = [(self.servo_curr_pos[1]/10 * math.pi)/180,
                                   (self.servo_curr_pos[2]/10 * math.pi)/180,
                                   (self.servo_curr_pos[3]/10 * math.pi)/180,
                                   (self.servo_curr_pos[4]/10 * math.pi)/180,
                                   (self.servo_curr_pos[5]/10 * math.pi)/180]
        self.state_arm.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.state_arm.effort = []

        self.state_gripper.name = ['gripper_finger_left', 'gripper_finger_right']
        self.state_gripper.position = [((self.servo_curr_pos[0]/10 * math.pi)/180)/-100,
                                       (-(self.servo_curr_pos[0]/10 * math.pi)/180)/-100]
        self.state_gripper.velocity = [0.0, 0.0]
        self.state_gripper.effort = []

        self.joint_state.publish(self.state_arm)
        self.joint_state.publish(self.state_gripper)

    def send_move_command(self):
        interrupt_wait = 0.01
        for s in range(0, self.servos):
            norm_position = int(abs(self.servo_new_pos[s]))
            self.pwm.setServoPulse(s,norm_position)
            #time.sleep(interrupt_wait)

        self.servo_curr_pos = self.servo_new_pos

        #rospy.loginfo(rospy.get_name() + ": Sending command to ra1 pro: " + cmd + "\n")
        self.send_robot_state()
        time.sleep(0.02)

    def move_direction(self, data):
        position = self.servo_curr_pos[data.servo] + data.direction
        if (abs(position) < self.servo_pos_max[data.servo]) and (abs(position) > self.servo_pos_min[data.servo]):
            rospy.loginfo(rospy.get_name() + ": Commit direction move command")
            self.servo_new_pos[data.servo] = position
            self.send_move_command()
        else:
            rospy.loginfo(rospy.get_name() + ": Furthest position reached")

    def move_position(self, data):
        if abs(data.position) <= self.servo_pos_max[data.servo] and abs(data.position) >= self.servo_pos_min[data.servo]:
            pos_round = round(data.position/10) * 10
            if pos_round != self.servo_curr_pos[data.servo]:
                rospy.loginfo(rospy.get_name() + ": Commit position move command")
                self.servo_new_pos[data.servo] = pos_round
                self.send_move_command()
        else:
            error = ": ERROR position out of bounds - position must be smaller than {0}".format(self.servo_pos_max[data.servo])
            rospy.logerr(rospy.get_name() + error)

    def sleep_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to SLEEP position")
        self.servo_new_pos = [1000, 1500, 1550, 1650, 1600, 1550]
        self.send_move_command()
        time.sleep(1.0)
        rospy.loginfo(rospy.get_name() + ": SLEEP position reached")

    def norm_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to NORMAL position")
        self.servo_new_pos = self.servo_pos_center
        self.send_move_command()
        time.sleep(1.0)
        rospy.loginfo(rospy.get_name() + ": NORMAL position reached")

    def home_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to my HOME position")
        self.servo_new_pos = [1000, 530, 430, 2700, 2800, 1700]
        self.send_move_command()
        time.sleep(1.0)
        rospy.loginfo(rospy.get_name() + ": HOME position reached")


if __name__ == '__main__':
    rospy.init_node('RA1_PRO', anonymous=False)
    try:
        Ra1Pro()
    except rospy.ROSInterruptException:
        pass
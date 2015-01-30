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

# Normal Usage:
# - Connect
# - Client sends "Ready"
# - Server sends "ON"
# - Server sends commands (Move)
# - ...
# - Server sends "OFF"

## Serial Protocol:
## SERVER -> CLIENT ##

# Send: "ON" => Init and Power On Motors (must be 2 char long!)
# Send: "OFF00000.." => Power Off Motors (must be 48 char long!)
# Send: "ADC00000.." => Make ADC Measurements (warning and emergency stops, 48 char long!)
# Send: "HARD0000.." => Turn On HARD mode (ADC but only warnings, need ADC ON to work, 48 char long!)
# Send: "..S3N200V2.." => Servo 3 to negative position 200 with velocity of 2 (must be 48 char long!)
# Send: "..S1P100V2.." => Servo 1 to positive position 100 with velocity of 2 (must be 48 char long!)

# S - Which servo 	(S1,S2,S3,S4,S5 or S6)
# N - The position 	(Min. N900 ..... P900 max)
# V - The speed		(Fast V0  ......  V10 slow)

# Complete Movement Samples:
# Send: "S1N200V2S2P000V2S3P000V2S4P000V2S5P000V2S6P000V2" => Moves all servos at once
# Send: "S1N200V2S2N200V2S3N200V2S4N200V2S5N200V2S6N200V2"
# Send: "S1P100V1S2N300V2S3P400V3S4N500V4S5P400V5S6N300V6"

## CLIENT -> SERVER ##
# Send: "Alfred # wait ON" => Robot is powered on and ready for "ON", repeats until ON cmd
# Send: "Alfred # ON init" => On command was send
# Send: "Alfred # ON executed" => On command has finished
# Send: "Alfred # OFF init" => Off command was send
# Send: "Alfred # OFF executed" => Off command has finished
# Send: "Alfred # MOVE init" => Move command was send
# Send: "Alfred # MOVE executed" => Move command has finished
# Send: "Alfred # ADC init" => ADC command was send
# Send: "Alfred # ADC executed" => ADC command has finished
# Send: "Alfred # ADC switched OFF" => ADC mode switched off
# Send: "Alfred # ADC switched ON" => ADC mode switched ON
# Send: "Alfred # HARD init" => HARD command was send
# Send: "Alfred # HARD executed" => HARD command has finished
# Send: "Alfred # HARD switched OFF" => Hard mode switched off
# Send: "Alfred # HARD switched ON" => Hard mode switched ON
# Send: "Alfred # COMMAND Error" => Wrong command style
# Send: "Alfred # CURRENT 2 error" => Current of servo 2 too high, powers off (need reset)
# Send: "Alfred # CURRENT 1 warning" => Current of servo 1 was close to maximum


import roslib; roslib.load_manifest('ra1_pro')
import rospy
import serial
import time
import math

from std_msgs.msg import *
from sensor_msgs.msg import JointState
from ra1_pro_msgs.msg import *
from ra1_pro_msgs.srv import *

wait_to_send = 1


class Ra1Pro:

    def __init__(self):
        rospy.loginfo(rospy.get_name() + ": Starting Node")
        rospy.loginfo(rospy.get_name() + ": Waiting for start")

        self.driver_pub = rospy.Publisher('ra1_pro/feedback', String, queue_size=10)
        self.joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.basic_cmd_service = rospy.Service('ra1_pro_cmd', BasicCMD, self.handle_basic_cmd)

        rospy.Subscriber("/ra1_pro/cmd", Ra1ProSimpleMove, self.simple_movement)
        rospy.Subscriber("/move_group/controller_joint_states", JointState, self.get_trajectory)

        rate = rospy.Rate(10)

        self.ser = serial.Serial()
        self.ser.timeout = 0

        rospy.on_shutdown(self.cleanup)

        self.msg_not_ready = ": System is not ready, did you start?"

        # 1 gripper
        # 2 rotate gripper
        # 3 joint of gripper
        # 4 joint (weak)
        # 5 joint (big one)
        # 6 base (rotation)
        self.servos = 6
        # save servo positions
        #                      1    2    3    4    5    6
        self.servo_curr_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_new_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_prev_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_speed = [0, 0, 0, 0, 0, 0]

        self.servo_name = ['gripper_finger_left', 'servo_2', 'servo_3', 'servo_4', 'servo_5', 'servo_6']

        # less than these values
        #self.servo_max_pos = [900, 800, 500, 900, 900, 900]
        self.servo_max_pos = [900, 900, 900, 900, 900, 900]

        self.state_arm = JointState()
        self.state_gripper = JointState()
        self.header = Header()

        self.ready = False
        # what was read on the serial
        self.connected = False

        while not rospy.is_shutdown():
            if self.ser.isOpen() == 1:
                self.send_robot_state()
                try:
                    serial_read = self.ser.readline().rstrip()
                    string = rospy.get_name() + ": Serial Read: %s" % serial_read
                    #rospy.loginfo(string)
                    self.check_response(str(serial_read))
                    self.driver_pub.publish(String(string))
                except:
                    if self.connected is False:
                        rospy.loginfo(rospy.get_name() + ": Port is closed")
                    else:
                        rospy.logerr(rospy.get_name() + ": Port is closed!")
            rate.sleep()

    def handle_basic_cmd(self, req):
        resp = BasicCMDResponse()

        if req.cmd == req.STOP:
            if self.ready is True:
                self.ready = False
                self.cleanup()
                rospy.loginfo(rospy.get_name() + ": Closing")
                resp = resp.SUCCESS
            else:
                rospy.logerr(rospy.get_name() + self.msg_not_ready)
                resp = resp.ERROR
        elif req.cmd == req.START:
            if self.ready is not True:
                self.init_serial()
                if self.ser.isOpen() == 1:
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

    def simple_movement(self, data):
        if data.direction == 0 and self.ready is True:
            self.move_position(data)
        elif data.direction > 0 and self.ready is True:
            self.move_direction(data)
        else:
            rospy.logerr(rospy.get_name() + self.msg_not_ready)

    def get_trajectory(self, data):
        if self.ready is False:
            rospy.logerr(rospy.get_name() + self.msg_not_ready)
            return

        new_position = False
        #servo = data.name[0]
        #rospy.loginfo(rospy.get_name() + ": Got trajectory")

        servo_start = 0
        servo_end = len(data.name)

        #gripper = False

        #if servo == "gripper_finger_left":
        #    servo_start = 0
        #    servo_end = 1
        #    gripper = True

        for s in range(servo_start, servo_end):
            index = self.servo_name.index(data.name[s])
            #print "servo: " + self.servo_name[index] + " index: " + str(index)
            # remap from rad to grad and scale to hundred
            #norm_position = ((data.position[s-1]*180/math.pi)*10)
            norm_position = ((data.position[s]*180/math.pi)*10)
            pos_round = round(norm_position)

            # gripper movement must be extra scaled
            if index is 0:
                pos_round *= -100

            #speed = round(abs(data.velocity[s-1]*10))
            #if speed < 1:
            #    speed = 1
            #self.servo_speed[s] = speed

            if abs(pos_round) <= self.servo_max_pos[index]:
                if pos_round != self.servo_curr_pos[index]:
                    self.servo_new_pos[index] = pos_round
                    new_position = True
            else:
                error = ": ERROR position {0} servo {1} out of bounds - position must be smaller than {2}".format(round(norm_position), index, self.servo_max_pos[index])
                self.servo_new_pos[index] = math.copysign(self.servo_max_pos[index], pos_round)
                rospy.logerr(rospy.get_name() + error)

        if new_position:
            self.send_move_command()
            #rospy.loginfo(rospy.get_name() + ": Commit position move command")

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
        cmd = ''
        for s in range(0, self.servos):
            norm_position = str(int(abs(self.servo_new_pos[s]))).rjust(3, '0')
            if self.servo_new_pos[s] < 0.0:
                single = "S{0}N{1}V{2}".format((s+1), norm_position, int(self.servo_speed[s]))
            else:
                single = "S{0}P{1}V{2}".format((s+1), norm_position, int(self.servo_speed[s]))
            cmd = cmd + single

        self.servo_prev_pos = self.servo_curr_pos
        self.servo_curr_pos = self.servo_new_pos

        #rospy.loginfo(rospy.get_name() + ": Sending command to ra1 pro: " + cmd + "\n")
        self.send_serial(cmd)
        self.send_robot_state()

    def move_direction(self, data):
        position = self.servo_curr_pos[(data.servo-1)] + data.direction
        if abs(position) < self.servo_max_pos[(data.servo-1)]:
            rospy.loginfo(rospy.get_name() + ": Commit direction move command")
            self.servo_new_pos[(data.servo - 1)] = position
            self.send_move_command()
        else:
            rospy.loginfo(rospy.get_name() + ": Furthest position reached")

    def move_position(self, data):
        if abs(data.position) <= self.servo_max_pos[(data.servo-1)]:
            pos_round = round(data.position/100) * 100
            if pos_round != self.servo_curr_pos[(data.servo-1)]:
                rospy.loginfo(rospy.get_name() + ": Commit position move command")
                self.servo_new_pos[(data.servo - 1)] = pos_round
                self.send_move_command()
        else:
            error = ": ERROR position out of bounds - position must be smaller than {0}".format(self.servo_max_pos[(data.servo-1)])
            rospy.logerr(rospy.get_name() + error)

    def sleep_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to SLEEP position")
        self.servo_new_pos = [-400.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_move_command()
        self.send_serial("OFF")
        rospy.loginfo(rospy.get_name() + ": SLEEP position reached")

    def norm_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to NORMAL position")
        self.servo_new_pos = [-400.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_move_command()
        rospy.loginfo(rospy.get_name() + ": NORMAL position reached")

    def home_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to my HOME position")
        self.servo_new_pos = [-400.0, 780.0, 800.0, -700.0, -600.0, -440.0]
        self.send_move_command()
        rospy.loginfo(rospy.get_name() + ": HOME position reached")

    def send_serial(self, string):
        norm_string = string
        if len(string) < 48:
            # adding zeros if the msg is not long enough
            norm_string = string.ljust(48, '0')
        try:
            self.ser.flushInput()
            self.ser.write(norm_string)
        except:
            error = ": ERROR the port is maybe not open!"
            rospy.logerr(rospy.get_name() + error)
        rospy.sleep(0.07)

    def init_serial(self):
        rospy.loginfo(rospy.get_name() + ": INIT serial connection")
        self.ser.baudrate = 38400
        self.ser.port = '/dev/ttyUSB0'
        try:
            if self.ser.isOpen() == 0:
                self.ser.open()
        except:
            rospy.logerr(rospy.get_name() + ": Cannot Open Port!")

        # Try to start the robot as long as init occurs
        while self.connected == 0:
            rospy.loginfo(rospy.get_name() + ": Serial connecting...")
            #TODO maybe test something longer, case its already on
            self.send_serial("ON")
            time.sleep(wait_to_send)
            self.send_serial("HARD")
        rospy.loginfo(rospy.get_name() + ": Serial connection established")

    def cleanup(self):
        if self.ser.isOpen() == 1:
            self.sleep_position()
            self.ser.close()


if __name__ == '__main__':
    rospy.init_node('RA1_PRO', anonymous=False)
    try:
        Ra1Pro()
    except rospy.ROSInterruptException:
        pass
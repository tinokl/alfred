#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com
# RA1-Pro-AREXX Velocity Command

# Subscribes on topic "vel_cmd" with Ra1ProVelCmd messages
# Publishes connection information on topic "ra1_pro" with strings

## Ra1ProVelCmd:
#Header header
#string command
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

from std_msgs.msg import String
from ra1_pro_msgs.msg import Ra1ProVelCmd

wait_to_send = 2

class RA1_PRO:

    def __init__(self):
        rospy.loginfo(rospy.get_name() + ": Starting Node")
        self.driver_pub = rospy.Publisher('ra1_pro', String)
        rospy.Subscriber("vel_cmd", Ra1ProVelCmd, self.check_command)

        self.ser = serial.Serial()
        rospy.on_shutdown(self.cleanup)

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
        self.servo_speed = [2, 2, 2, 2, 2, 2]

        # less than these values
        self.servo_max_pos = [3, 8, 5, 9, 9, 9]
        self.ready = False
        # what was read on the serial
        self.connected = False

        while not rospy.is_shutdown():
            if self.ser.isOpen() == 1:
                serial_read = self.ser.readline().rstrip()
                string = "Serial Read: %s" % serial_read
                rospy.loginfo(string)
                self.check_response(str(serial_read))
                self.driver_pub.publish(String(string))
            rospy.sleep(0.1)

    def check_response(self, string):
        cmd_on = "Alfred # ON executed"
        cmd_off = "Alfred # OFF executed"

        if string == cmd_on:
            self.connected = True
        if string == cmd_off:
            self.connected = False

    def check_command(self, data):
        cmd = data.command
        if cmd == "CLOSE":
            self.ready = False
            self.cleanup()
        elif cmd == "START":
            self.init_serial()
            if self.ser.isOpen() == 1:
                self.norm_position()
                self.ready = True
        elif cmd == "POS" and self.ready:
            if self.ser.isOpen() == 1:
                self.move_position(data)
        elif cmd == "DIR" and self.ready:
            if self.ser.isOpen() == 1:
                self.move_direction(data)

    def send_move_command(self):
        cmd = ''
        for s in range(0, self.servos):
            if self.servo_new_pos[s] < 0:
                single = "S{0}N{1}V{2}".format((s+1), abs(self.servo_new_pos[s]), self.servo_speed[s])
            else:
                single = "S{0}P{1}V{2}".format((s+1), abs(self.servo_new_pos[s]), self.servo_speed[s])
            cmd = cmd.join(single)
        self.servo_prev_pos = self.servo_curr_pos
        self.servo_curr_pos = self.servo_new_pos
        self.send_serial(cmd)

    def move_direction(self, data):
        position = self.servo_curr_pos[(data.servo-1)] + data.direction
        if abs(position) < self.servo_max_pos[(data.servo-1)]:
            rospy.loginfo(rospy.get_name() + ": Commit direction move command")
            self.servo_new_pos[(data.servo - 1)] = position
            self.send_move_command()
        else:
            rospy.loginfo(rospy.get_name() + ": Furthest position reached")

    def move_position(self, data):
        if data.position < self.servo_max_pos[(data.servo-1)]:
            if data.position != self.servo_pos[(data.servo-1)]:
                rospy.loginfo(rospy.get_name() + ": Commit position move command")
                self.servo_new_pos[(data.servo - 1)] = data.position
                self.send_move_command()
            else:
                error = ": ERROR position out of bounds - position must be smaller than {0}".format(self.servo_max_pos[(data.servo-1)])
                rospy.loginfo(rospy.get_name() + error)

    def sleep_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to SLEEP position")
        time.sleep(wait_to_send)
        #self.send_serial("S1N500V2S2N300V2S3N600V2S4N000V2S5N000V2S6N400V2")
        time.sleep(wait_to_send)
        self.send_serial("S1N200V2S2P000V2S3P000V2S4P000V2S5P000V2S6P000V2")
        time.sleep(wait_to_send)
        self.send_serial("OFF")
        time.sleep(wait_to_send)
        rospy.loginfo(rospy.get_name() + ": SLEEP position reached")

    def norm_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to NORMAL position")
        time.sleep(wait_to_send)
        self.send_serial("S1N200V2S2P000V2S3P000V2S4P000V2S5P000V2S6P000V2")
        time.sleep(wait_to_send)
        rospy.loginfo(rospy.get_name() + ": NORMAL position reached")

    def send_serial(self, string):
        norm_string = string
        if len(string) < 48:
            norm_string = string.ljust(48, '0')
        self.ser.write(norm_string)

    def init_serial(self):
        rospy.loginfo(rospy.get_name() + ": INIT serial connection")
        self.ser.baudrate = 38400
        self.ser.port = '/dev/ttyUSB0'
        if self.ser.isOpen() == 0:
            self.ser.open()

        # Try to start the robot as long as init occurs
        while self.connected == False:
            rospy.loginfo(rospy.get_name() + ": Serial connecting...")
            self.send_serial("ON")
            time.sleep(wait_to_send)
        rospy.loginfo(rospy.get_name() + ": Serial connection established")

    def cleanup(self):
        if self.ser.isOpen() == 1:
            self.sleep_position()
            self.ser.close()


if __name__ == '__main__':
    rospy.init_node('RA1_PRO', anonymous=False)
    try:
        RA1_PRO()
    except rospy.ROSInterruptException:
        pass
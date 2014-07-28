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

## Serial Protocoll:
## SERVER -> CLIENT ##
# Send: "ON000000" => Init and Power On Motors (must be 8 char long)
# Send: "OFF00000" => Power Off Motors (must be 8 char long)
# Send: "R" => If Current was to high, reset robot
# Send: "S3N200V2" => Servo 3 to negative position 200 with velocity of 2
# Send: "S1P100V2" => Servo 1 to positive position 100 with velocity of 2

# S - Which servo   (S0,S1,S2,S3,S4,S5 or S6)
# N - The position  (Min. N900 ..... P900 max)
# V - The speed     (Fast V0  ......  V10 slow)

## CLIENT -> SERVER ##
# Send: "Alfred # Ready" => Robot is powered on and ready for "ON"
# Send: "Alfred # ON init" => On command was send
# Send: "Alfred # ON executed" => On command has finished
# Send: "Alfred # OFF init" => Off command was send
# Send: "Alfred # OFF executed" => Off command has finished
# Send: "Alfred # MOVE init" => Move command was send
# Send: "Alfred # MOVE executed" => Move command has finished
# Send: "Alfred # COMMAND Error" => Wrong command style
# Send: "Alfred # CURRENT 2 error" => Current of servo 2 too high, powers off (need reset)
# Send: "Alfred # CURRENT 1 warning" => Current of servo 1 was close to maximum
# Send: "Alfred # need RESET" => Robot was reset because of high current, RESET musst be send
# Send: "Alfred # RESET init" => Reset command was send
# Send: "Alfred # RESET executed" => Reset command has finished

    ############################ OLD OLD OLD
    # Commands:
    # START  ->  inits serial, moves the arm to the start position
    # CLOSE  ->  moves the arm to a sleeping position, closes serial
    # Move Command 1 direction: MCD + servo + direction (the higher the further)
    # Move Command 2 position: MCP + clockwise/counterclockwise + servo + position

    # Examples
    # Move Command 1, Servo 6, clockwise 2
    # string command = MCD
    # int8 servo = 6
    # int8 direction = 2
    # int8 clockwise = 1
    # int8 position = 0  -> unused

    # Move Command 2, Servo 3, counter-clockwise 2
    # string command = MCP
    # int8 servo = 3
    # int8 direction = 0 -> unused
    # int8 clockwise = 0 -> unused
    # int8 position = 3  -> unused

    # Simple Command
    # string command = START
    # rest is unused or not checked


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
        # save servo positions
        #                  1    2    3    4    5    6
        self.servo_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
            self.norm_position()
            self.ready = True
        elif cmd == "POS" and self.ready:
            if self.ser.isOpen() == 1:
                self.move_position(data)
        elif cmd == "DIR" and self.ready:
            if self.ser.isOpen() == 1:
                self.move_direction(data)

    def move_direction(self, data):
        position = self.servo_pos[(data.servo-1)] + data.direction
        if abs(position) < self.servo_max_pos[(data.servo-1)]:
            rospy.loginfo(rospy.get_name() + ": Commit direction move command")
            if position < 0:
                send = "S{0}N{1}V2".format(data.servo, abs(position))
            else:
                send = "S{0}P{1}V2".format(data.servo, abs(position))
            self.send_serial(send)
        else:
            rospy.loginfo(rospy.get_name() + ": Furthest position reached")

    def move_position(self, data):
        if data.position < self.servo_max_pos[(data.servo-1)]:
            if data.position != self.servo_pos[(data.servo-1)]:
                rospy.loginfo(rospy.get_name() + ": Commit position move command")
                if position < 0:
                    send = "S{0}N{1}V2".format(data.servo, data.position)
                else:
                    send = "S{0}P{1}V2".format(data.servo, data.position)
                self.send_serial(send)
            else:
                error = ": ERROR position out of bounds - position musst be smaller than {0}".format(self.servo_max_pos[(data.servo-1)])
                rospy.loginfo(rospy.get_name() + error)

    def sleep_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to SLEEP position")
        time.sleep(wait_to_send)
        self.send_serial("S1N500V2")
        time.sleep(wait_to_send)
        self.send_serial("S6N400V2")
        time.sleep(wait_to_send)
        self.send_serial("S2N300V2")
        time.sleep(wait_to_send)
        self.send_serial("S3N600V2")
        time.sleep(wait_to_send)
        self.send_serial("S2N700V2")
        time.sleep(wait_to_send)
        self.send_serial("S4P999V2")
        time.sleep(wait_to_send)
        self.send_serial("S5N000V2")
        time.sleep(wait_to_send)
        self.send_serial("OFF00000")
        time.sleep(wait_to_send)
        rospy.loginfo(rospy.get_name() + ": SLEEP position reached")

    def norm_position(self):
        rospy.loginfo(rospy.get_name() + ": Turn to NORMAL position")
        time.sleep(wait_to_send)
        self.send_serial("S1N500V2")
        time.sleep(wait_to_send)
        self.send_serial("S6N400V2")
        time.sleep(wait_to_send)
        self.send_serial("S2N300V2")
        time.sleep(wait_to_send)
        self.send_serial("S3N600V2")
        time.sleep(wait_to_send)
        self.send_serial("S2N700V2")
        time.sleep(wait_to_send)
        self.send_serial("S4P999V2")
        time.sleep(wait_to_send)
        self.send_serial("S5N000V2")
        time.sleep(wait_to_send)
        rospy.loginfo(rospy.get_name() + ": NORMAL position reached")

    def send_serial(self, string):
        self.ser.write(string)
        # Save new current position
        if string[0] != 'O' and string != 'OFF00000' and string != 'R':
            if string[2] == 'N':
                self.servo_pos[int(string[1])-1] = (-1) * float(string[3])
            else:
                self.servo_pos[int(string[1])-1] = float(string[3])

    def init_serial(self):
        rospy.loginfo(rospy.get_name() + ": INIT serial connection")
        self.ser.baudrate = 38400
        self.ser.port = '/dev/ttyUSB0'
        if self.ser.isOpen() == 0:
            self.ser.open()

        # Try to start the robot as long as init occours
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
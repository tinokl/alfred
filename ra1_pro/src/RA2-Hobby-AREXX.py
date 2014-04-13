#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# RA2-Hobby-AREXX Velocity Command

# Subscribes on topic "velocity_commands" with arm_vel_msg messages
# Publishes connection information on topic "RA2_hobby" with strings

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

import roslib; roslib.load_manifest('eva')
import rospy
import serial
import time

from std_msgs.msg import String
from eva.msg import arm_vel_msg

wait_to_send = 1

class RA2Hobby:

  def __init__(self):

    rospy.loginfo(rospy.get_name() + ": Starting Node")
    self.RA2_Hobby_pub = rospy.Publisher('RA2_Hobby', String)
    rospy.Subscriber("velocity_commands", arm_vel_msg, self.checkCommand)

    self.ser = serial.Serial()
    rospy.on_shutdown(self.cleanup)

    # 1 gripper
    # 2 rotate gripper
    # 3 joint of gripper
    # 4 joint (weak)
    # 5 joint (big one)
    # 6 base (rotation)
    # save servo positions
    #                 1 2 3 4 5 6
    self.servo_pos = [0,0,0,0,0,0]
    # less than these values
    self.servo_max_pos = [3,8,5,9,9,9]
    self.ready = False

    while not rospy.is_shutdown():
      if self.ser.isOpen() == 1:
        line = self.ser.readline()
        str = "Serial Read: %s" % line  # lint:ok
        rospy.loginfo(str)
        self.RA2_Hobby_pub.publish(String(str))
      rospy.sleep(1.0)


  def checkCommand(self, data):
    command = data.command
    if command == "CLOSE":
      self.ready = False
      self.cleanup()
    elif command == "START":
      self.initSerial()
      self.normPosition()
      self.ready = True
    elif command == "MCP" and self.ready == True:
      if self.ser.isOpen() == 1:
        self.moveCommandPosition(data)
    elif command == "MCD" and self.ready == True:
      if self.ser.isOpen() == 1:	
        self.moveCommandDirection(data)


  def moveCommandDirection(self, data):
    position = self.servo_pos[(data.servo-1)]
    if data.clockwise == 1:
      position = position + data.direction
    else:
      position = position - data.direction	
    if abs(position) < self.servo_max_pos[(data.servo-1)]:
      rospy.loginfo(rospy.get_name() + ": Commit direction move command")
      str = "#{0}{1}{2}".format(data.clockwise, data.servo, abs(position)) # lint:ok
      self.sendSerial(str)
    else:
      rospy.loginfo(rospy.get_name() + ": Furthest position reached")


  def moveCommandPosition(self, data):
    if data.position < self.servo_max_pos[(data.servo-1)]:
      if data.position != self.servo_pos[(data.servo-1)]:
        rospy.loginfo(rospy.get_name() + ": Commit position move command")
        str = "#{0}{1}{2}".format(data.clockwise, data.servo, data.position)  # lint:ok
        self.sendSerial(str)
    else:
      error = ": ERROR position out of bounds - position musst be smaller than {0}".format(self.servo_max_pos[(data.servo-1)])
      rospy.loginfo(rospy.get_name() + error)


  def sleepPosition(self):
    rospy.loginfo(rospy.get_name() + ": Turn to SLEEP position")
    self.sendSerial("#060")
    time.sleep(wait_to_send)
    self.sendSerial("#048")
    time.sleep(wait_to_send)
    self.sendSerial("#058")
    time.sleep(wait_to_send)
    self.sendSerial("#134")
    time.sleep(wait_to_send)
    self.sendSerial("#012")
    time.sleep(wait_to_send)
    self.sendSerial("#000")
    time.sleep(wait_to_send)
    rospy.loginfo(rospy.get_name() + ": SLEEP position reached")


  def normPosition(self):
    rospy.loginfo(rospy.get_name() + ": Turn to NORMAL position")
    self.sendSerial("#100")
    time.sleep(wait_to_send)
    self.sendSerial("#060")
    time.sleep(wait_to_send)
    self.sendSerial("#048")
    time.sleep(wait_to_send)
    self.sendSerial("#058")
    time.sleep(wait_to_send)
    self.sendSerial("#132")
    time.sleep(wait_to_send)
    self.sendSerial("#122")
    time.sleep(wait_to_send)
    self.sendSerial("#012")
    time.sleep(wait_to_send)
    rospy.loginfo(rospy.get_name() + ": NORMAL position reached")


  def sendSerial(self, string):
    self.ser.write(string)
    if string != '#000' or string != '#100':
      if string[1] == '0':
        self.servo_pos[int(string[2])-1] = (-1) * int(string[3])
      else:
        self.servo_pos[int(string[2])-1] = int(string[3])


  def initSerial(self):
    rospy.loginfo(rospy.get_name() + ": INIT serial connection")
    self.ser.baudrate = 38400
    self.ser.port = '/dev/ttyUSB0'
    if self.ser.isOpen() == 0:
      self.ser.open()


  def cleanup(self):
    if self.ser.isOpen() == 1:
      self.sleepPosition()
      self.ser.close()


if __name__ == '__main__':
  rospy.init_node('RA2_Hobby_AREXX', anonymous=False)
  try:
    RA2Hobby()
  except rospy.ROSInterruptException:
    pass


#!/usr/bin/env python
import roslib; roslib.load_manifest('eva')
import rospy

from eva.msg import arm_vel_msg
from hark_msgs.msg import HarkSource
from std_msgs.msg import String
from sensor_msgs.msg import Joy
#from trajectory_msgs import JointTrajectory
#from trajectory_msgs import JointTrajectoryPoint

class cortex:

  def __init__(self):
    rospy.loginfo(rospy.get_name() + ": Starting Node")
    self.vel_com_pub = rospy.Publisher('velocity_commands', arm_vel_msg)
    rospy.Subscriber("joy", Joy, self.gotControl, queue_size = 10)
    rospy.Subscriber("HarkSource", HarkSource, self.heardNoise)
    rospy.Subscriber("speech", String, self.heardVoice)

    rospy.sleep(1.0)
    self.msg = arm_vel_msg( "START" , 0 , 0 , 0 , 0 )
    self.vel_com_pub.publish(self.msg)

    while not rospy.is_shutdown():
      rospy.sleep(1.0)

  def gotControl(self, data):
    joy = data
    # start button pressed
    if (joy.buttons[7] == 1):
      self.msg = arm_vel_msg( "START" , 0 , 0 , 0 , 0 )
      self.vel_com_pub.publish(self.msg)
      rospy.loginfo(rospy.get_name() + ": Got Controller Command - Sending Start")
    # back button pressed
    if (joy.buttons[6] == 1):
      self.msg = arm_vel_msg( "CLOSE" , 0 , 0 , 0 , 0 )
      self.vel_com_pub.publish(self.msg)
      rospy.loginfo(rospy.get_name() + ": Got Controller Command - Sending Stop")
    # left trigger horizontal
    if (joy.axes[0] > 0.9):
      self.msg = arm_vel_msg( "MCD" , 6 , 1 , 1 , 0 )
      self.vel_com_pub.publish(self.msg)
      rospy.loginfo(rospy.get_name() + ": Got Controller Command - Moving Joint")
      rospy.sleep(0.5)
    if (joy.axes[0] < -0.9):
      self.msg = arm_vel_msg( "MCD" , 6 , 1 , 0 , 0 )
      self.vel_com_pub.publish(self.msg)
      rospy.loginfo(rospy.get_name() + ": Got Controller Command - Moving Joint")
      rospy.sleep(0.5)

  def heardVoice(self, data):
    heard = data.data
    str_start = "start"
    str_stop = "stop"
    if (heard.find(str_start) != -1):
      self.msg = arm_vel_msg( "START" , 0 , 0 , 0 , 0 )
      self.vel_com_pub.publish(self.msg)
      rospy.loginfo(rospy.get_name() + ": Heard Voice Command - Sending Start")
    elif (heard.find(str_stop) != -1):
      self.msg = arm_vel_msg( "CLOSE" , 0 , 0 , 0 , 0 )
      self.vel_com_pub.publish(self.msg)
      rospy.loginfo(rospy.get_name() + ": Heard Voice Command - Sending Stop")


  def heardNoise(self, data):
    if data.exist_src_num > 0:
      theta = int(round(data.src[0].theta,0))
      if theta > 0:
        if theta < 95:
          self.msg = arm_vel_msg( "MCP" , 6 , 0 , 1 , (theta/10) )
          self.vel_com_pub.publish(self.msg)
          rospy.loginfo(rospy.get_name() + ": I want to turn %s" % str(theta/10))
      else:
        if theta > -95:
          self.msg = arm_vel_msg( "MCP" , 6 , 0 , 0 , ((-1)*theta/10) )
          self.vel_com_pub.publish(self.msg)
          rospy.loginfo(rospy.get_name() + ": I want to turn %s" % str(theta/10))


if __name__ == '__main__':
  rospy.init_node('cortex')
  try:
    cortex()
  except rospy.ROSInterruptException:
    pass



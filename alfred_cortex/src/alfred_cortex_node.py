#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com
# Alfred Main Cortex

import roslib; roslib.load_manifest('alfred_cortex')
import rospy
import smach
import smach_ros

from smach_ros import *
from std_msgs.msg import *
from ra1_pro_msgs.msg import *
from ra1_pro_msgs.srv import *
#from hark_msgs.msg import HarkSource

cmd_start = "START"
cmd_stop = "STOP"


# State Foo
class Foo(smach.State):

    def __init__(self, outcomes=['outcome1', 'outcome2']):

    def execute(self, userdata):
        return 'outcome1'


# State Bar
class Bar(smach.State):

    def __init__(self, outcomes=['outcome3', 'outcome4']):

    def execute(self, userdata):
        return 'outcome4'


# State Bas
class Bas(smach.State):

    def __init__(self, outcomes=['outcome5']):

    def execute(self, userdata):
        return 'outcome5'


class Cortex:

    def __init__(self):
        rospy.loginfo(rospy.get_name() + ": Awaking")

        #rospy.Subscriber("joy", Joy, self.gotControl, queue_size = 10)
        #rospy.Subscriber("HarkSource", HarkSource, self.heardNoise)
        #rospy.Subscriber("speech", String, self.heardVoice)

        #self.ra1_service_call(cmd_start)

        #while not rospy.is_shutdown():
        #    rospy.sleep(0.1)
        # Open the container

        # Create the top level state machine
        sm_top = smach.StateMachine(outcomes=['outcome5'])

        with sm_top:

            smach.StateMachine.add('BAS', Bas(),
                                   transitions={'outcome3':'SUB'})

            # Create the sub state machine
            sm_sub = smach.StateMachine(outcomes=['outcome4'])

            with sm_sub:

                smach.StateMachine.add('FOO', Foo(),
                                       transitions={'outcome1':'BAR',
                                                    'outcome2':'outcome4'})
                smach.StateMachine.add('BAR', Bar(),
                                       transitions={'outcome1':'FOO'})

            smach.StateMachine.add('SUB', sm_sub,
                                   transitions={'outcome4':'outcome5'})

    def ra1_service_call(self, command):
        rospy.wait_for_service('ra1_pro_cmd')
        try:
            ra1_pro_cmd = rospy.ServiceProxy('ra1_pro_cmd', BasicCMD)

            req = BasicCMDRequest()
            if command == cmd_start:
                response = ra1_pro_cmd(req.START)
            elif command == cmd_stop:
                response = ra1_pro_cmd(req.STOP)

            print response
        except rospy.ServiceException, e:
            rospy.logerr(rospy.get_name() + ": Service call failed: " + e)


        # def gotControl(self, data):
        #   joy = data
        #   # start button pressed
        #   if (joy.buttons[7] == 1):
        #     self.msg = arm_vel_msg( "START" , 0 , 0 , 0 , 0 )
        #     self.vel_com_pub.publish(self.msg)
        #     rospy.loginfo(rospy.get_name() + ": Got Controller Command - Sending Start")
        #   # back button pressed
        #   if (joy.buttons[6] == 1):
        #     self.msg = arm_vel_msg( "CLOSE" , 0 , 0 , 0 , 0 )
        #     self.vel_com_pub.publish(self.msg)
        #     rospy.loginfo(rospy.get_name() + ": Got Controller Command - Sending Stop")
        #   # left trigger horizontal
        #   if (joy.axes[0] > 0.9):
        #     self.msg = arm_vel_msg( "MCD" , 6 , 1 , 1 , 0 )
        #     self.vel_com_pub.publish(self.msg)
        #     rospy.loginfo(rospy.get_name() + ": Got Controller Command - Moving Joint")
        #     rospy.sleep(0.5)
        #   if (joy.axes[0] < -0.9):
        #     self.msg = arm_vel_msg( "MCD" , 6 , 1 , 0 , 0 )
        #     self.vel_com_pub.publish(self.msg)
        #     rospy.loginfo(rospy.get_name() + ": Got Controller Command - Moving Joint")
        #     rospy.sleep(0.5)

        # def heardVoice(self, data):
        #   heard = data.data
        #   str_start = "start"
        #   str_stop = "stop"
        #   if (heard.find(str_start) != -1):
        #     self.msg = arm_vel_msg( "START" , 0 , 0 , 0 , 0 )
        #     self.vel_com_pub.publish(self.msg)
        #     rospy.loginfo(rospy.get_name() + ": Heard Voice Command - Sending Start")
        #   elif (heard.find(str_stop) != -1):
        #     self.msg = arm_vel_msg( "CLOSE" , 0 , 0 , 0 , 0 )
        #     self.vel_com_pub.publish(self.msg)
        #     rospy.loginfo(rospy.get_name() + ": Heard Voice Command - Sending Stop")


        # def heardNoise(self, data):
        #   if data.exist_src_num > 0:
        #     theta = int(round(data.src[0].theta,0))
        #     if theta > 0:
        #       if theta < 95:
        #         self.msg = arm_vel_msg( "MCP" , 6 , 0 , 1 , (theta/10) )
        #         self.vel_com_pub.publish(self.msg)
        #         rospy.loginfo(rospy.get_name() + ": I want to turn %s" % str(theta/10))
        #     else:
        #       if theta > -95:
        #         self.msg = arm_vel_msg( "MCP" , 6 , 0 , 0 , ((-1)*theta/10) )
        #         self.vel_com_pub.publish(self.msg)
        #         rospy.loginfo(rospy.get_name() + ": I want to turn %s" % str(theta/10))


if __name__ == '__main__':
    rospy.init_node('Cortex')
    try:
        Cortex()
    except rospy.ROSInterruptException:
        pass



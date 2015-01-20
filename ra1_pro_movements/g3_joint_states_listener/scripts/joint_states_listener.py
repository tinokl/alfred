#!/usr/bin/env python
#spins off a thread to listen for joint_states messages
#and provides the same information (or subsets of) as a service

import roslib
import sys
import rospy
from g3_joint_states_listener_msgs.srv import *
from sensor_msgs.msg import JointState
import threading


#holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self):
        rospy.init_node('joint_states_listener')
        self.lock = threading.Lock()
        self.name = []
        self.position = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        s = rospy.Service('robot/joint_states_filtered', returnJointStates, self.return_joint_states)


    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('robot/joint_states', JointState, self.joint_states_callback)
        rospy.spin()


    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        self.name = msg.name
        self.position = msg.position
        self.lock.release()


    #returns (found, position, velocity, effort) for the joint joint_name
    #(found is 1 if found, 0 otherwise)
    def return_joint_state(self, joint_name):

        #no messages yet
        if self.name == []:
            rospy.logerr("No robot_state messages received!\n")
            return (0, 0., 0., 0.)

        found = False

        for count in range(1,10):
            #return info for this joint
            self.lock.acquire()
            if joint_name in self.name:
                index = self.name.index(joint_name)
                position = self.position[index]
                self.lock.release()
                found = True
                break

            #unless it's not found
            else:
                self.lock.release()
                rospy.logerr("Joint %s not found yet!", (joint_name,))
                rospy.sleep(0.01)
                # return (0, 0., 0., 0.)

        if found:
            return (1, position)
        else:
            return (0, 0.)


    #server callback: returns arrays of position, velocity, and effort
    #for a list of joints specified by name
    def return_joint_states(self, req):
        (joint_found, position) = self.return_joint_state(req.name)
        return returnJointStatesResponse(joint_found, position)


#run the server
if __name__ == "__main__":

    latestjointstates = LatestJointStates()

    print "joints_states_listener server started, waiting for queries"
    rospy.spin()

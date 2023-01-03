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

def limit_value(value, min_value, max_value):
    return max(min_value, min(value, max_value))


class Servo:
    def __init__(self, **kwargs):

        # 500 == 0 degree
        # 1000 == 45 degree
        # 1500 == 90 degree
        # 2000 == 135 degree
        # 2500 == 180 degree

        self.name = kwargs.get('name', 'servo')
        self.pos_min = kwargs.get('pos_min', 300)
        self.pos_max = kwargs.get('pos_max', 2000)
        self.pos_norm = kwargs.get('pos_norm', 1000)
        self.vel_max = kwargs.get('vel_max', 1000)

        self.pos_current = 0
        self.pos_goal = 0
        self.moveing = False

    def update_goal_position(self, new_pos):
        # check given limits
        if abs(new_pos) > self.pos_max or abs(new_pos) < self.pos_min:
            return False

        # round position to servo limits aka resolution
        new_pos_round = round(new_pos/10) * 10

        # check if we need to move
        if new_pos_round != self.pos_current:
            self.moveing = True
            self.pos_goal = new_pos_round # new target to reach
        else:
            self.moveing = False

        return True

    def update_current_position(self, new_pos):
        # round position to servo limits aka resolution
        new_pos_round = round(new_pos/10) * 10

        self.pos_current = new_pos_round     
        # check if we need to move
        if self.pos_current != self.pos_goal:
            self.moveing = True
        else:
            self.moveing = False


class Ra1Pro:

    def __init__(self):
        self.msg_not_ready = ": System is not ready, what?"

        rospy.loginfo(rospy.get_name() + ": Starting Node")

        self.offline = rospy.get_param('~offline')
        self.ready = False
        self.connected = False

        self.driver_pub = rospy.Publisher('ra1_pro/feedback', String, queue_size=10)
        self.joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.basic_cmd_service = rospy.Service('ra1_pro_cmd', BasicCMD, self.handle_basic_cmd)

        rospy.Subscriber("/ra1_pro/cmd", Ra1ProSimpleMove, self.callback_simple_movement)
        rospy.Subscriber("/ra1_pro/callback_neural_link", Ra1ProNeuralLink, self.callback_neural_link)
        rospy.Subscriber("/move_group/controller_joint_states", JointState, self.get_trajectory)

        rate = rospy.Rate(10)

        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        self.pwd_set = True

        self.robot = []

        self.servo_gripper = Servo(name='gripper', pos_min=950, pos_max=1650, pos_norm=1000, vel_max=1000)
        self.robot.append(self.servo_gripper)

        self.servo_wrist = Servo(name='wrist', pos_min=505, pos_max=2405, pos_norm=1500, vel_max=10)
        self.robot.append(self.servo_wrist)

        self.servo_servo3 = Servo(name='servo3', pos_min=430, pos_max=2500, pos_norm=1550, vel_max=10)
        self.robot.append(self.servo_servo3)

        self.servo_arm = Servo(name='arm', pos_min=500, pos_max=2700, pos_norm=1650, vel_max=10)
        self.robot.append(self.servo_arm)

        self.servo_shoulder = Servo(name='shoulder', pos_min=700, pos_max=2800, pos_norm=1600, vel_max=10)
        self.robot.append(self.servo_shoulder)

        self.servo_base = Servo(name='base', pos_min=500, pos_max=2500, pos_norm=1550, vel_max=10)
        self.robot.append(self.servo_base)
        
        rospy.loginfo(rospy.get_name() + ": Added following servos to robot:")
        for i, servo in enumerate(self.robot):
            rospy.loginfo(rospy.get_name() + ": Added servo {}: {}".format(i + 1, servo.name))

        self.set_position_sleep()

        """

        # save servo positions
        #                      0    1    2    3    4    5
        self.servo_curr_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_new_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # less than these values
        self.servo_name = ["gripper", "wrist", "servo3", "arm", "shoulder", "base"]
        self.servo_pos_max = [1650, 2405, 2500, 2700, 2800, 2500]
        self.servo_pos_center = [1000, 1500, 1550, 1650, 1600, 1550]
        self.servo_pos_min = [950, 505, 430, 500, 700, 500]
        self.servo_speed_limit = [1000, 10, 10, 10, 10, 10]

        """

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
            rospy.loginfo(rospy.get_name() + ": Closing")
            rospy.sleep(2)
            self.move_position_sleep()
            resp = resp.SUCCESS
        elif req.cmd == req.START:
            if self.ready is not True:
                if self.pwd_set or self.offline:
                    rospy.loginfo(rospy.get_name() + ": Starting")
                    self.move_position_norm()
                    self.ready = True
                    rospy.sleep(2)
                    self.move_position_home()
                    resp = resp.SUCCESS
            else:
                rospy.logerr(rospy.get_name() + self.msg_not_ready)
                resp = resp.ERROR
        return resp

    def callback_simple_movement(self, data):
        if data.direction == 0 and self.ready is True:
            self.move_position(data)
        elif data.direction > 0 and self.ready is True:
            self.move_direction(data)
        else:
            rospy.logerr(rospy.get_name() + self.msg_not_ready)

    def callback_neural_link(self, data):
        if self.ready is True:

            for i, servo in enumerate(self.robot):
                # Normalize neural input within servo min max range
                new_pos = servo.pos_min + (servo.pos_max - servo.pos_min) * getattr(data, 'servo_{}_state'.format(i + 1))
                feedback = servo.update_goal_position(new_pos)
                if feedback == False:
                    rospy.logwarn(rospy.get_name() + ": neural command failed!")

            self.execute_move_command()

            """
            self.servo_new_pos[0] = servo_pos_min[0] + (servo_pos_max[0]-servo_pos_min[0]) * data.servo_1_state
            self.servo_new_pos[1] = servo_pos_min[1] + (servo_pos_max[1]-servo_pos_min[1]) * data.servo_2_state
            self.servo_new_pos[2] = servo_pos_min[2] + (servo_pos_max[2]-servo_pos_min[2]) * data.servo_3_state
            self.servo_new_pos[3] = servo_pos_min[3] + (servo_pos_max[3]-servo_pos_min[3]) * data.servo_4_state
            self.servo_new_pos[4] = servo_pos_min[4] + (servo_pos_max[4]-servo_pos_min[4]) * data.servo_5_state
            self.servo_new_pos[5] = servo_pos_min[5] + (servo_pos_max[5]-servo_pos_min[5]) * data.servo_6_state
            """

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
            self.execute_move_command()

    def send_robot_state(self):
        self.state_arm = JointState()
        self.state_gripper = JointState()

        self.header = Header()
        self.header.stamp = rospy.Time.now()
        self.header.frame_id = '/base'

        self.state_arm.header = self.header
        self.state_gripper.header = self.header

        self.state_arm.name = ['servo_2', 'servo_3', 'servo_4', 'servo_5', 'servo_6']
        self.state_arm.position = [(self.robot[1].pos_current/10 * math.pi)/180,
                                   (self.robot[2].pos_current/10 * math.pi)/180,
                                   (self.robot[3].pos_current/10 * math.pi)/180,
                                   (self.robot[4].pos_current/10 * math.pi)/180,
                                   (self.robot[5].pos_current/10 * math.pi)/180]
        self.state_arm.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.state_arm.effort = []

        self.state_gripper.name = ['gripper_finger_left', 'gripper_finger_right']

        self.state_gripper.position = [((self.servo_gripper.pos_current/10 * math.pi)/180)/-100,
                                       (-(self.servo_gripper.pos_current/10 * math.pi)/180)/-100]

        self.state_gripper.velocity = [0.0, 0.0]
        self.state_gripper.effort = []

        self.joint_state.publish(self.state_arm)
        self.joint_state.publish(self.state_gripper)

        """
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
        """

    def execute_move_command(self):
        #interrupt_wait = 0.01

        execution_running = True
        execution_start = rospy.Time.now()
        execution_max_duration = rospy.Duration.from_sec(5)

        # TODO - Should we run this first check all the first servos are done?
        # TODO - At start the initial current pos is wrong, any other behaviour?
        while execution_running:
            for i, servo in enumerate(self.robot):
                #update_position = int(abs(servo.pos_goal))
                diff_position = servo.pos_goal - servo.pos_current
                # limit to the maximal allowed position change
                step_size = min(servo.vel_max, abs(diff_position))
                step = math.copysign(step_size, diff_position)
                update_position = servo.pos_current + step
                self.pwm.setServoPulse(i, update_position)
                servo.update_current_position(update_position)
                #time.sleep(interrupt_wait)

            self.send_robot_state()

            # check if all servos reached the destination
            reached_destination = True
            for servo in self.robot:
                if servo.moveing:
                    reached_destination = False
            if reached_destination:
                execution_running = False

            # check on timeout
            execution_now = rospy.Time.now()
            execution_duration = execution_now - execution_start
            if execution_duration > execution_max_duration:
                error = ": ERROR execution of movement reached timout: {0} seconds!".format(execution_max_duration)
                rospy.logerr(rospy.get_name() + error)
                execution_running = False

            time.sleep(0.02)

        """
        interrupt_wait = 0.01
        for s in range(0, self.servos):
            norm_position = int(abs(self.servo_new_pos[s]))
            self.pwm.setServoPulse(s,norm_position)
            #time.sleep(interrupt_wait)

        self.servo_curr_pos = self.servo_new_pos

        #rospy.loginfo(rospy.get_name() + ": Sending command to ra1 pro: " + cmd + "\n")
        self.send_robot_state()
        time.sleep(0.02)

        """

    def move_direction(self, data):
        print("TODO implement")
        return
        """
        position = self.servo_curr_pos[data.servo] + data.direction
        if (abs(position) < self.servo_pos_max[data.servo]) and (abs(position) > self.servo_pos_min[data.servo]):
            rospy.loginfo(rospy.get_name() + ": Commit direction move command")
            self.servo_new_pos[data.servo] = position
            self.execute_move_command()
        else:
            rospy.loginfo(rospy.get_name() + ": Furthest position reached")
        """

    def move_position(self, data):
        servo = self.servos[data.servo]  # Get the Servo object from the list
        feedback = servo.update_goal_position(data.position)
        if feedback == True:
            rospy.loginfo(rospy.get_name() + ": Commit position move command")
            self.execute_move_command()
        else:
            rospy.logwarn(rospy.get_name() + ": Commit position move command failed!")

    """ LEGACY
    def move_position(self, data):
        if abs(data.position) <= self.servo_pos_max[data.servo] and abs(data.position) >= self.servo_pos_min[data.servo]:
            pos_round = round(data.position/10) * 10
            if pos_round != self.servo_curr_pos[data.servo]:
                rospy.loginfo(rospy.get_name() + ": Commit position move command")
                self.servo_new_pos[data.servo] = pos_round
                self.execute_move_command()
        else:
            error = ": ERROR position out of bounds - position must be smaller than {0}".format(self.servo_pos_max[data.servo])
            rospy.logerr(rospy.get_name() + error)
    """

    def move_position_norm(self):
        rospy.loginfo(rospy.get_name() + ": Turn to NORMAL position")

        for servo in self.robot:
            feedback = servo.update_goal_position(servo.pos_norm)

        self.execute_move_command()
        time.sleep(1.0)
        rospy.loginfo(rospy.get_name() + ": NORMAL position reached")

    def set_position_sleep(self):
        rospy.loginfo(rospy.get_name() + ": Set initial position to: SLEEP")
        new_servo_pos = [1000, 1500, 1550, 1650, 1600, 1550]

        for i, servo in enumerate(self.robot):
            servo.pos_current = new_servo_pos[i]

    def move_position_sleep(self):
        rospy.loginfo(rospy.get_name() + ": Turn to SLEEP position")
        new_servo_pos = [1000, 1500, 1550, 1650, 1600, 1550]

        for i, servo in enumerate(self.robot):
            feedback = servo.update_goal_position(new_servo_pos[i])

        self.execute_move_command()
        time.sleep(1.0)
        rospy.loginfo(rospy.get_name() + ": SLEEP position reached")

    def move_position_home(self):
        rospy.loginfo(rospy.get_name() + ": Turn to my HOME position")
        new_servo_pos = [1000, 530, 430, 2700, 2800, 1700]

        for i, servo in enumerate(self.robot):
            feedback = servo.update_goal_position(new_servo_pos[i])

        self.execute_move_command()
        time.sleep(1.0)
        rospy.loginfo(rospy.get_name() + ": HOME position reached")


if __name__ == '__main__':
    rospy.init_node('RA1_PRO', anonymous=False)
    try:
        Ra1Pro()
    except rospy.ROSInterruptException:
        pass
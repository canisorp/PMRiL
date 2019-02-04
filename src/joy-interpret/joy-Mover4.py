#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np


VELOCITY_VALUE = 50

class CommandToJointState:

    def __init__(self):

        rospy.Subscriber('joint_states', JointState, self.joints_callback, queue_size=10)
        rospy.Subscriber('joy', Joy, self.command_callback, queue_size=10)

        self.joint_pub = rospy.Publisher('/CPRMoverJointVel',JointState, queue_size=1)
        self.joint_pubCom = rospy.Publisher('/CPRMoverCommands', String, queue_size=1)

        self.joint_state = JointState()

        self.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


        self.string = String()

    def joints_callback(self, msg):
        self.joint_state.position[0] = msg.position[0]
        self.joint_state.position[1] = msg.position[1]
        self.joint_state.position[2] = msg.position[2]
        self.joint_state.position[3] = msg.position[3]

        if(msg.position[0] > 2.5 and self.joint_state.velocity[0] > 0):
            self.joint_state.velocity[0] = 0.0
            self.joint_pub.publish(self.joint_state)

        if(msg.position[0] < -1.05 and self.joint_state.velocity[0] < 0):
            self.joint_state.velocity[0] = 0.0
            self.joint_pub.publish(self.joint_state)

        if(msg.position[1] > 1.03 and self.joint_state.velocity[1] > 0):
            self.joint_state.velocity[1] = 0.0
            self.joint_pub.publish(self.joint_state)

        if(msg.position[1] < -0.48 and self.joint_state.velocity[1] < 0):
            self.joint_state.velocity[1] = 0.0
            self.joint_pub.publish(self.joint_state)

        if(msg.position[2] > 2.35 and self.joint_state.velocity[2] > 0):
            self.joint_state.velocity[2] = 0.0
            self.joint_pub.publish(self.joint_state)

        if(msg.position[2] < -0.65 and self.joint_state.velocity[2] < 0):
            self.joint_state.velocity[2] = 0.0
            self.joint_pub.publish(self.joint_state)

        if(msg.position[3] > 2.0 and self.joint_state.velocity[3] > 0):
            self.joint_state.velocity[3] = 0.0
            self.joint_pub.publish(self.joint_state)

        if(msg.position[3] < -2.0 and self.joint_state.velocity[3] < 0):
            self.joint_state.velocity[3] = 0.0
            self.joint_pub.publish(self.joint_state)


    def command_callback(self, msg):

        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # enable motoros
        if(msg.buttons[0] == 1 ):
            self.string='Enable'

        # gripper manipulation
        if(msg.buttons[1] == 1 ):
            self.string='GripperOpen'

        if(msg.buttons[2] == 1 ):
            self.string='GripperClose'

        # reset motors
        if(msg.buttons[3] == 1 ):
            self.string='Reset'

        # connect Mover4
        if(msg.buttons[8] == 1):
            self.string ='Connect'

        # Jonit 0
        if(msg.axes[0]>0 and self.joint_state.position[0]<2.5):

            self.joint_state.velocity[0] = VELOCITY_VALUE

        elif(msg.axes[0]<0 and self.joint_state.position[0]>-1.05):

            self.joint_state.velocity[0] = -VELOCITY_VALUE

        else:
             self.joint_state.velocity[0] = 0.0

        # from bottom to top
        # Joint 1
        if(msg.axes[1]>0 and self.joint_state.position[1]<1.03):

            self.joint_state.velocity[1] = VELOCITY_VALUE

        elif(msg.axes[1]<0 and self.joint_state.position[1]>-0.40):

            self.joint_state.velocity[1] = -VELOCITY_VALUE

        else:
            self.joint_state.velocity[1] = 0.0

        # Joint 2
        if(msg.axes[3]>0 and self.joint_state.position[2]<2.35):

            self.joint_state.velocity[2] = VELOCITY_VALUE

        elif(msg.axes[3]<0 and self.joint_state.position[2]>-0.65):

            self.joint_state.velocity[2] = -VELOCITY_VALUE

        else:
            self.joint_state.velocity[2] = 0.0

        # Joint 3
        if(msg.axes[5]>0 and self.joint_state.position[3]<2.0):

            self.joint_state.velocity[3] = VELOCITY_VALUE

        elif(msg.axes[5]<0 and self.joint_state.position[3]>-2.0):

            self.joint_state.velocity[3] = -VELOCITY_VALUE
        else:
            self.joint_state.velocity[3] = 0.0

        self.joint_state.header.stamp = rospy.Time.now()

        self.joint_pub.publish(self.joint_state)

        if(self.string is not None):
            self.joint_pubCom.publish(self.string)
        self.string=None


if __name__ == '__main__':

    rospy.init_node('command_to_joint_state')

    command_to_joint_state = CommandToJointState()

    rospy.spin()

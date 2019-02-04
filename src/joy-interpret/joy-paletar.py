#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from threading import Timer

class JoyInterpreter(object):

    def __init__(self):
        rospy.init_node("joy_interpreter")

        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=10)

        # Publisher
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Inicijalne postavke
        self.axis_linear = rospy.get_param('axis_linear', 1)
        self.axis_angular = rospy.get_param('axis_angular', 2)
        self.scale_linear = rospy.get_param('scale_linear', 0.2)
        self.scale_angular = rospy.get_param('scale_angular', 0.9)

        self.last_published = Twist()

        rospy.Timer(rospy.Duration(0.1), self.publish)

        rospy.spin()


    def joy_callback(self, msg):
        vel = Twist()

        vel.linear.x = (self.scale_linear + 0.1) * msg.axes[self.axis_linear]
        vel.angular.z = self.scale_angular * msg.axes[self.axis_angular]

        self.last_published = vel

    def publish(self, event):
        self.pub_cmd.publish(self.last_published)


if __name__ == '__main__':
    try:
        JoyInterpreter()
    except rospy.ROSInterruptException:
        pass

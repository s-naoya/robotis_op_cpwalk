#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D


class JoyControl(object):
    def __init__(self):
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.cmdctrl_pub = rospy.Publisher(
            "/robotis_op/cmd_ctrl", Int32, queue_size=10)
        self.cmdpos_pub = rospy.Publisher(
            "/robotis_op/cmd_pos", Pose2D, queue_size=10)

    def joy_callback(self, msg):
        self.pose = Pose2D()
        self.ctrl = Int32()
        if msg.buttons[14] == 1.0:  # cross button
            self.ctrl.data = 1
        elif msg.buttons[13] == 1.0:  # circle button
            self.ctrl.data = 2
        elif msg.buttons[15] == 1.0:  # square button
            self.ctrl.data = 3
            self.cmdctrl_pub.publish(self.ctrl)
            rospy.signal_shutdown("shutdown")
        else:
            self.ctrl.data = 4
        self.cmdctrl_pub.publish(self.ctrl)

        self.pose.x = msg.axes[1] * 0.02  # left joystick upwards
        self.pose.y = msg.axes[0] * 0.02  # left joystick leftwards
        if msg.buttons[11] == 1.0:  # rear right 1 button
            self.pose.theta = 10.0
        elif msg.buttons[10] == 1.0:  # rear left 1 button
            self.pose.theta = -10.0
        else:
            self.pose.theta = 0.0
        self.cmdpos_pub.publish(self.pose)
        # print "x: ", self.pose.x, ", y: ", self.pose.y, ", theta: ", self.pose.theta, ", ctrl: ", self.ctrl.data


if __name__ == '__main__':
    rospy.init_node("walking_pattern_generator_ctrl")
    joyctrl = JoyControl()
    rospy.spin()

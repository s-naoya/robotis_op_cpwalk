#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D


def main():
    rospy.init_node("walking_pattern_generator_ctrl")
    cmdctrl_pub = rospy.Publisher("/robotis_op/cmd_ctrl", Int32, queue_size=10)
    cmdpos_pub = rospy.Publisher("/robotis_op/cmd_pos", Pose2D, queue_size=10)
    pose = Pose2D()
    ctrl = Int32()
    pose.x = 0.0
    pose.y = 0.0
    pose.theta = 0.0

    while not rospy.is_shutdown():
        print "stop: 1, start: 2, finish: 3, front: w, right: d, left: a, back: s, turn r: e, turn l: q"
        cmd = raw_input(">> ")
        if cmd == "1" or cmd == "2" or cmd == "3":
            ctrl = int(cmd)
            cmdctrl_pub.publish(ctrl)
            if cmd == "3":
                break
        elif cmd == "w":
            pose.x += 0.02
            cmdpos_pub.publish(pose)
        elif cmd == "d":
            pose.y -= 0.01
            cmdpos_pub.publish(pose)
        elif cmd == "a":
            pose.y += 0.01
            cmdpos_pub.publish(pose)
        elif cmd == "s":
            pose.x -= 0.02
            cmdpos_pub.publish(pose)
        elif cmd == "e":
            pose.theta += 2.0
            cmdpos_pub.publish(pose)
        elif cmd == "q":
            pose.theta -= 2.0
            cmdpos_pub.publish(pose)

if __name__ == '__main__':
    main()

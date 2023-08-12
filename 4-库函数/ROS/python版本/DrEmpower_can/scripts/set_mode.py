#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_set_mode
import DrEmpower as b

def callback(data):
    b.set_mode(id_num=data.id, mode=data.mode)
    rospy.loginfo("mode of motor Num.%s is set as %s" , data.id, data.mode)


def listener():
    rospy.init_node("set_mode_one_actuator", anonymous=True)
    rospy.Subscriber("set_mode", arg_set_mode, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


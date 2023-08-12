#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_set_zero_position
import DrEmpower as b

def callback(data):
    b.set_zero_position(id_num=data.id)
    rospy.loginfo("zero position of motor Num.%s is set" , data.id)


def listener():
    rospy.init_node("set_zero_position_one_actuator", anonymous=True)
    rospy.Subscriber("set_zero_position", arg_set_zero_position, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


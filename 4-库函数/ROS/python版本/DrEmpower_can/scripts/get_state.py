#!/usr/bin/env python

import time
import rospy
from DrEmpower_can.msg import arg_get_state
import DrEmpower as b

def callback(data):
    state = []
    state = b.get_state(id_num=data.id)
    rospy.loginfo("motor Num.%s is read, angle: %s; speed: %s" , data.id, state[0], state[1])


def listener():
    rospy.init_node("get_state_one_actuator", anonymous=True)
    rospy.Subscriber("get_state", arg_get_state, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


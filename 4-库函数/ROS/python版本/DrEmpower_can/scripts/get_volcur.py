#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_get_volcur
import DrEmpower as b

def callback(data):
    volcur = []
    volcur = b.get_volcur(id_num=data.id)
    rospy.loginfo("motor Num.%s is read, volue: %s; currrent: %s" , data.id, volcur[0], volcur[1])


def listener():
    rospy.init_node("get_volcur_one_actuator", anonymous=True)
    rospy.Subscriber("get_volcur", arg_get_volcur, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


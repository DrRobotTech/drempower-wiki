#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_estop
import DrEmpower as b

def callback(data):
    b.estop(id_num=data.id)
    rospy.loginfo("motor Num.%s is stopped" , data.id)


def listener():
    rospy.init_node("estop_one_actuator", anonymous=True)
    rospy.Subscriber("estop", arg_estop, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


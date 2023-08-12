#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_clear_error
import DrEmpower as b

def callback(data):
    b.clear_error(id_num=data.id)
    rospy.loginfo("error of motor Num.%s is cleared" , data.id)


def listener():
    rospy.init_node("clear_error_one_actuator", anonymous=True)
    rospy.Subscriber("clear_error", arg_clear_error, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

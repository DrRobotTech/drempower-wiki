#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_set_torque
import DrEmpower as b

def callback(data):
    b.set_torque(id_num=data.id, torque=data.torque, param=data.param, mode=data.mode)
    rospy.loginfo("motor Num.%s is set, torque: %s, param: %s, mode: %s" , data.id, data.torque, data.param, data.mode)


def listener():
    rospy.init_node("set_torque_one_actuator", anonymous=True)
    rospy.Subscriber("set_torque", arg_set_torque, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


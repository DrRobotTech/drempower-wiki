#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_set_torques
import DrEmpower as b

def callback(data):
    b.set_torques(id_list=data.id_list, torque_list=data.torque_list, param=data.param, mode=data.mode)
    rospy.loginfo("motor Nums.%s are set, torques: %s, param: %s, mode: %s" , data.id_list, data.torque_list, data.param, data.mode)


def listener():
    rospy.init_node("set_torques_more_actuator", anonymous=True)
    rospy.Subscriber("set_torques", arg_set_torques, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


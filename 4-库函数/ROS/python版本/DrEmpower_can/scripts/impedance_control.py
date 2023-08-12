#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_impedance_control
import DrEmpower as b

def callback(data):
    b.impedance_control(id_num=data.id, pos=data.pos, vel=data.vel, tff=data.tff, kp=data.kp, kd=data.kd)
    rospy.loginfo("motor Num.%s is set, pos: %s; vel: %s; tff: %s; kp: %s; kd: %s" , data.id, data.pos, data.vel, data.tff, data.kp, data.kp)


def listener():
    rospy.init_node("impedance_control_one_actuator", anonymous=True)
    rospy.Subscriber("impedance_control", arg_impedance_control, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


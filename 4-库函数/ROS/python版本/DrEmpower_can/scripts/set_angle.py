#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_set_angle
import DrEmpower as b

def callback(data):
    b.set_angle(id_num=data.id, angle=data.angle, speed=data.speed, param=data.param, mode=data.mode)
    rospy.loginfo("motor Num.%s is set, angle: %s; speed: %s; param: %s; mode: %s" , data.id, data.angle, data.speed, data.param, data.mode)


def listener():
    rospy.init_node("set_agnle_one_actuator", anonymous=True)
    rospy.Subscriber("set_angle", arg_set_angle, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


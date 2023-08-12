#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_step_angle
import DrEmpower as b

def callback(data):
    b.step_angle(id_num=data.id, angle=data.angle, speed=data.speed, param=data.param, mode=data.mode)
    rospy.loginfo("motor Num.%s is set, step_angle: %s; speed: %s; param: %s; mode: %s" , data.id, data.angle, data.speed, data.param, data.mode)


def listener():
    rospy.init_node("step_agnle_one_actuator", anonymous=True)
    rospy.Subscriber("step_angle", arg_step_angle, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


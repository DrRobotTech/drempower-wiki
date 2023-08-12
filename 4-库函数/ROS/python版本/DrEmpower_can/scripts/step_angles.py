#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_step_angles
import DrEmpower as b

def callback(data):
    b.step_angles(id_list=data.id_list, angle_list=data.angle_list, speed=data.speed, param=data.param, mode=data.mode)
    rospy.loginfo("motor Nums.%s are set, step_angles: %s; speed: %s; param: %s; mode: %s" , data.id_list, data.angle_list, data.speed, data.param, data.mode)


def listener():
    rospy.init_node("step_agnles_more_actuator", anonymous=True)
    rospy.Subscriber("step_angles", arg_step_angles, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


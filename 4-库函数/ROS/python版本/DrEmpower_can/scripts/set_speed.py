#!/usr/bin/env python

import rospy
# from std_msgs.msg import Float32
from DrEmpower_can.msg import arg_set_speed
import DrEmpower as b

def callback(data):
    b.set_speed(id_num=data.id, speed=data.speed, param=data.param, mode=data.mode)
    rospy.loginfo("motor Num.%s is set, speed: %s, param: %s, mode: %s" , data.id, data.speed, data.param, data.mode)
# def callback(data):
#     b.set_speed(id_num=0, speed=data.data)
#     rospy.loginfo(rospy.get_caller_id() + " speed is set as %s" , data.data)


def listener():
    rospy.init_node("set_speed_one_actuator", anonymous=True)
    rospy.Subscriber("set_speed", arg_set_speed, callback)
    rospy.spin()
# def listener():
#     rospy.init_node("set_speed_one_actuator", anonymous=True)
#     rospy.Subscriber("set_speed", Float32, callback)
#     rospy.spin()


if __name__ == '__main__':
    listener()


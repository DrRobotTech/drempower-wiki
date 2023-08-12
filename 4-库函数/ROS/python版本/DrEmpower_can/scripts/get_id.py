#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_get_id
import DrEmpower as b

def callback(data):
    print("get_id")
    id = b.get_id(id_num=data.id)
    rospy.loginfo("id of motor is %s" , id)


def listener():
    rospy.init_node("get_id_one_actuator", anonymous=True)
    rospy.Subscriber("get_id", arg_get_id, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_set_id
import DrEmpower as b

def callback(data):
    b.set_id(id_num=data.id_old, new_id=data.id_new)
    rospy.loginfo("id of motor Num.%s is set as %s" , data.id_old, data.id_new)


def listener():
    rospy.init_node("set_id_one_actuator", anonymous=True)
    rospy.Subscriber("set_id", arg_set_id, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


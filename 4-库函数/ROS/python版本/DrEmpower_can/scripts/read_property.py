#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_read_property
import DrEmpower as b

def callback(data):
    property = b.read_property(id_num=data.id, property=data.property)
    rospy.loginfo("%s of motor Num.%s is %s" , data.property, data.id, property)


def listener():
    rospy.init_node("read_property_one_actuator", anonymous=True)
    rospy.Subscriber("read_property", arg_read_property, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


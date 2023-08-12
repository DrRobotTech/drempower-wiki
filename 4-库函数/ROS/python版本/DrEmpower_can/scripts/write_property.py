#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_write_property
import DrEmpower as b

def callback(data):
    b.write_property(id_num=data.id, property=data.property, value=data.value)
    rospy.loginfo("%s of motor Num.%s is set as %s" , data.property, data.id, data.value)


def listener():
    rospy.init_node("write_property_one_actuator", anonymous=True)
    rospy.Subscriber("write_property", arg_write_property, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


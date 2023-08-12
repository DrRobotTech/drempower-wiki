#!/usr/bin/env python3

import rospy
from DrEmpower_can.msg import arg_dump_error
import DrEmpower as b

def callback(data):
    b.dump_error(id_num=data.id)
    # rospy.loginfo("error of motor Num.%s is cleared" , data.id)


def listener():
    rospy.init_node("dump_error_one_actuator", anonymous=True)
    rospy.Subscriber("dump_error", arg_dump_error, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

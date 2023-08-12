#!/usr/bin/env python

import rospy
from DrEmpower_can.msg import arg_set_uart_baud_rate
import DrEmpower as b

def callback(data):
    b.set_uart_baud_rate(id_num=data.id, baud_rate=data.baud_rate)
    # boad_rate = b.get_(id_num=data.id)
    rospy.loginfo("boad_rate of motor No.%s is set as %s" , data.id, data.baud_rate)


def listener():
    rospy.init_node("set_uart_baud_rate_one_actuator", anonymous=True)
    rospy.Subscriber("set_uart_baud_rate", arg_set_uart_baud_rate, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()


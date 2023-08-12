#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def talker():
    rospy.init_node("talker", anonymous=True)
    pub = rospy.Publisher("speed", Float32, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        speed = 20
        # rospy.loginfo(hello_str)
        pub.publish(speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInternalException:
        pass

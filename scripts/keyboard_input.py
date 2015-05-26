#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import time

def talker():
    pub = rospy.Publisher('command', Int32)
    rospy.init_node('keyboard_input', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        st = raw_input("Enter command: ")
        try:
            st = int(st)
            pub.publish(st)
            print "sent intput"
        except ValueError:
            pass
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *

def talker():
    pub = rospy.Publisher('paduck/cmd_vel', Twist, queue_size=10)
    rospy.init_node('pabuck', anonymous=True)
    rate = rospy.Rate(0.05)

    while not rospy.is_shutdown():
        num = Twist()
        num.linear.x = 0.3
        pub.publish(num)
        rate.sleep()
        num.linear.x = -0.3
        pub.publish(num)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
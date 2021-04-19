#!/usr/bin/env python
# Provides a helper node for converting max speed and curvature values to twist messages
# Author: Gowtham Ram Pagadala

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

max_speed_ = 2
curvature_ = 0

def processCurvature(msg):
    global curvature_
    curvature_ = msg.data

def processMaxSpeed(msg):
    global max_speed_
    max_speed_ = msg.data

def conversion():
    rospy.init_node('speedCurve_to_twist_node', anonymous=False)

    twist_pub_ = rospy.Publisher('~cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber("~curvature", Float64, processCurvature)
    rospy.Subscriber("~max_speed", Float64, processMaxSpeed)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = max_speed_
        twist.angular.z = max_speed_*curvature_
        twist_pub_.publish(twist)
        rate.sleep()


if __name__ == '__main__':    
    try:
        conversion()
    except rospy.ROSInterruptException:
        pass

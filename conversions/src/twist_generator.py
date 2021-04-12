#!/usr/bin/env python
# Provides a helper node for converting max speed and curvature values to twist messages
# Author: Gowtham Ram Pagadala

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

max_speed_ = 0
curvature_ = 0

def processCurvature(msg):
    global curvature_
    curvature_ = msg.data

def processMaxSpeed(msg):
    global max_speed_
    max_speed_ = msg.data

def conversion():
    rospy.init_node('twist_output_node', anonymous=True)

    twist_pub_ = rospy.Publisher('twist_output', Twist, queue_size=10)
    
    rospy.Subscriber("pure_pursiut_curvature", Float32, processCurvature)
    rospy.Subscriber("max_speed", Float32, processMaxSpeed)

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
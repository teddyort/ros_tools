#!/usr/bin/env python
# Provides a helper node for converting speed and curvature values to twist messages
# Author: Gowtham Ram Pagadala

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class SpeedCurvatureToTwistNode(object):
    """Converts a given speed and curvature to a Twist for use in a cmd_vel """

    def __init__(self):
        """ Initialize the node """
        self.node_name = rospy.get_name()

        # Initialize variables
        self.curvature = None

        # Setup the publisher and subscribers
        self.twist_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("~curvature", Float64, self.process_curvature)
        rospy.Subscriber("~speed", Float64, self.process_speed)

    def process_curvature(self, msg):
        """ Handle receiving a curvature command """
        self.curvature = msg.data

    def process_speed(self, msg):
        """ Handle receiving a speed command """
        speed = msg.data
        if not self.curvature is None:
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = speed * self.curvature
            self.twist_pub.publish(twist)


if __name__ == '__main__':
    # Initializing the node with the given name
    rospy.init_node('speed_curvature_to_twist_node')
    speed_curvature_to_twist_node = SpeedCurvatureToTwistNode()
    rospy.spin()
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import conversions.pose_converter as pc

import tf2_ros

class TfPathPublisherNode(object):
    """TfPathPublisherNode
    Author: Teddy Ort
    Inputs (tf):  Periodically queries the tf tree to track the desired transform
    Outputs (nav_msgs.msg.Path): Publishes a path containing the poses of the transforms
    """

    def __init__(self):
        """Initializes the node"""
        self.node_name = rospy.get_name()

        # Get parameters
        self.parent_frame = rospy.get_param("~parent_frame", 'map')
        self.child_frame = rospy.get_param("~child_frame", 'base_link')
        self.rate = rospy.get_param("~rate", 100)

        self.pub = rospy.Publisher("~path", Path, queue_size=1)

        # Init vars
        self.path = Path()
        self.path.header.frame_id = self.parent_frame

        # Setup the transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.loginfo("has started")

        rate = rospy.Rate(self.rate, reset=True)
        while not rospy.is_shutdown():
            rate.sleep()
            self.update_path()

    def update_path(self):
        """ Add the latest pose to the path and publish it"""
        try:
            ts = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            err = "[%s] Couldn't find transform %s->%s" \
                  % (self.node_name, self.parent_frame, self.child_frame)
            rospy.logwarn_throttle(5, err)
            return

        # Check if time moved backward, and if so, reset the path
        if self.path.poses and self.path.poses[-1].header.stamp > ts.header.stamp:
            self.path.poses = []

        # Add the new pose and publish
        self.path.poses.append(pc.to_pose_stamped(ts))
        self.path.header.stamp = rospy.Time.now()
        self.pub.publish(self.path)

    def loginfo(self, msg):
        rospy.loginfo("[%s] %s", self.node_name, msg)


if __name__ == '__main__':
    rospy.init_node('tf_path_publisher_node', anonymous=False)
    tf_path_publisher_node = TfPathPublisherNode()
    rospy.spin()

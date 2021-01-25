#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import tf2_ros

class TfPathPublisher(object):
    """TfPathPublisher
    Author: Teddy Ort
    Inputs:
    Outputs:
    """

    def __init__(self):
        """Initializes the node"""
        self.node_name = rospy.get_name()

        # Get parameters
        self.parent_frame = rospy.get_param("~parent_frame", 'map')
        self.child_frame = rospy.get_param("~child_frame", 'base_link')
        self.rate = rospy.get_param("~rate", 100)

        self.pub = rospy.Publisher("~/path", Path, queue_size=1)

        # Init vars
        self.path = Path()
        self.path.header.frame_id = self.parent_frame

        # Setup the transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.loginfo("has started")

        rate = rospy.Rate(self.rate)
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

        # Convert transform to pose
        msg = PoseStamped()
        msg.header = ts.header
        trans, rot = ts.transform.translation, ts.transform.rotation
        p, o = msg.pose.position, msg.pose.orientation
        p.x, p.y, p.z = trans.x, trans.y, trans.z
        o.x, o.y, o.z, o.w = rot.x, rot.y, rot.z, rot.w
        self.path.poses.append(msg)
        self.path.header.stamp = rospy.Time.now()
        self.pub.publish(self.path)

    def loginfo(self, msg):
        rospy.loginfo("[%s] %s", self.node_name, msg)


if __name__ == '__main__':
    rospy.init_node('tf_path_publisher', anonymous=False)
    tf_path_publisher = TfPathPublisher()
    rospy.spin()

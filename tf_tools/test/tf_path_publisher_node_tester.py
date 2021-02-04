#!/usr/bin/env python
import rospy
import unittest
import rostest
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
import tf2_ros

class TfPathPublisherNodeTester(unittest.TestCase):

    def setUp(self):
        # Setup the node
        rospy.init_node('tf_path_publisher_node_tester', anonymous=False)
        self.path_msg = None
        self.path_end = None

        # Setup the tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Setup the subscriber
        self.sub = rospy.Subscriber("~path", Path, self.callback)

        # Wait for the node  to finish starting up
        self.do_until_timeout(lambda: self.sub.get_num_connections() > 0)

    def test_publishers_and_subscribers(self):
        pass

    def test_pont_to_point(self):
        """ Test sending two different transforms and checking the resulting path """
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.rotation.w = 1 # Needed for valid quaternion
        self.br.sendTransform(tf)

        # Wait for a path to be received with the correct pose at the end
        self.do_until_timeout(lambda: self.path_end == (0,0))

        # Now send another location
        tf.transform.translation.x = 5
        tf.transform.translation.y = 10
        tf.header.stamp = rospy.Time.now()
        self.br.sendTransform(tf)

        # Wait for a path to be received with the correct pose at the end
        self.do_until_timeout(lambda: self.path_end == (5,10))

        # The full path should have the correct start and end
        start = self.path_msg.poses[0]
        end = self.path_msg.poses[-1]
        self.assertEqual(start.pose.position.x, 0)
        self.assertEqual(start.pose.position.y, 0)
        self.assertEqual(end.pose.position.x, 5)
        self.assertEqual(end.pose.position.y, 10)

    def callback(self, msg):
        self.path_msg = msg
        pos = msg.poses[-1].pose.position
        self.path_end = pos.x, pos.y

    def do_until_timeout(self, condition, msg='Test timed out', action=None, timeout=5, hz=10):
        # Wait for the condition or until the timeout passes
        rate = rospy.Rate(hz)
        timeout = rospy.Time.now() + rospy.Duration(timeout)
        while not condition() and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if action is not None:
                action()
            rate.sleep()
        self.assertLess(rospy.Time.now(), timeout, msg)


if __name__ == '__main__':
    rostest.rosrun('tf_tools', 'tf_path_publisher_node_tester', TfPathPublisherNodeTester)

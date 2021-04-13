#!/usr/bin/env python
import rosunit
import unittest
import numpy as np

import conversions.pose_converter as pc
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped

class PoseConverterTester(unittest.TestCase):
    def setUp(self):
        # Define some default values for testing
        self.position = (1, 1, 1)
        self.orientation = (0, 0, 1, 0)
        self.orientation2d = np.pi
        self.parent = 'map'
        self.child = 'base_link'

    def test_pose_roundtrip(self):
        pose = Pose()
        p, o = pose.position, pose.orientation
        (p.x, p.y, p.z) = self.position
        (o.x, o.y, o.z, o.w) = self.orientation
        self.assertEqual(pose, pc.to_pose(pose))

    def test_pose_stamped_roundtrip(self):
        ps = PoseStamped()
        p, o = ps.pose.position, ps.pose.orientation
        (p.x, p.y, p.z) = self.position
        (o.x, o.y, o.z, o.w) = self.orientation
        ps.header.frame_id = self.parent
        self.assertEqual(ps, pc.to_pose_stamped(ps))

    def test_transform_roundtrip(self):
        tf = Transform()
        p, o = tf.translation, tf.rotation
        (p.x, p.y, p.z) = self.position
        (o.x, o.y, o.z, o.w) = self.orientation
        self.assertEqual(tf, pc.to_transform(tf))

    def test_transform_stamped_roundtrip(self):
        tfs = TransformStamped()
        p, o = tfs.transform.translation, tfs.transform.rotation
        (p.x, p.y, p.z) = self.position
        (o.x, o.y, o.z, o.w) = self.orientation
        tfs.header.frame_id = self.parent
        tfs.child_frame_id = self.child
        self.assertEqual(tfs, pc.to_transform_stamped(tfs))

    def test_pose_transform_roundtrip(self):
        pose = Pose()
        p, o = pose.position, pose.orientation
        (p.x, p.y, p.z) = self.position
        (o.x, o.y, o.z, o.w) = self.orientation
        tf = pc.to_transform(pose)

        # Check that we got a transform
        self.assertTrue(isinstance(tf, Transform))

        # Now convert back to pose and compare
        self.assertEqual(pose, pc.to_pose(tf))

    def test_tuple_roundtrip(self):
        tup = (self.position, self.orientation)
        self.assertEqual(tup, pc.to_tuple(tup))

    def test_tuple2d_roundtrip(self):
        tup2d = self.position[0:2] + (self.orientation2d,)
        self.assertEqual(tup2d, pc.to_tuple2d(tup2d))


if __name__ == '__main__':
    rosunit.unitrun('rosutils', 'pose_converter_tester', PoseConverterTester)

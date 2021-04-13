#!/usr/bin/env python
import rosunit
import unittest
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from conversions import path_converter as pc


class PathConverterTester(unittest.TestCase):
    def setUp(self):
        # Define some default values for testing
        self.pose1 = ((0, 0, 0), (0, 0, 0, 1))
        self.pose2 = ((1, 1, 0), (0, 0, 1, 0))
        np.random.seed(42) # Use a fixed seed for test repeatability

    def test_path_roundtrip(self):
        path = Path()
        p1 = PoseStamped()
        p, o = p1.pose.position, p1.pose.orientation
        (p.x, p.y, p.z) = self.pose1[0]
        (o.x, o.y, o.z, o.w) = self.pose1[1]
        p2 = PoseStamped()
        p, o = p2.pose.position, p2.pose.orientation
        (p.x, p.y, p.z) = self.pose2[0]
        (o.x, o.y, o.z, o.w) = self.pose2[1]
        path.poses = [p1, p2]
        self.assertEqual(path, pc.to_path(path))

    def test_array_roundtrip(self):
        arr = np.random.rand(30, 7) # There are seven columns in a 3D pose
        np.testing.assert_allclose(arr, pc.to_array(arr))

    def test_array2d_roundtrip(self):
        arr = np.random.rand(30, 3) # There are three columns in a 2D pose
        np.testing.assert_allclose(arr, pc.to_array2d(arr))


if __name__ == '__main__':
    rosunit.unitrun('rosutils', 'path_converter_tester', PathConverterTester)

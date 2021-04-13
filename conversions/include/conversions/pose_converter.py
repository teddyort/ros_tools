# PoseConverter.py
# Provides a helper class for converting between/among common ros geometry types:
# Pose, PoseStamped, Transform, TransformStamped
# Author: Teddy Ort

import pprint
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from std_msgs.msg import Header
import tf.transformations as tr


class Converter(object):
    def __init__(self, obj=None):
        # Initialize empty class members
        self.parent_frame = ''
        self.child_frame = ''
        self.stamp = rospy.Time()
        self.position = (0.0, 0.0, 0.0)
        self.orientation = (0.0, 0.0, 0.0, 1.0)

        if isinstance(obj, Pose):
            self._from_pose(obj)
        elif isinstance(obj, PoseStamped):
            self._from_pose_stamped(obj)
        elif isinstance(obj, Transform):
            self._from_transform(obj)
        elif isinstance(obj, TransformStamped):
            self._from_transform_stamped(obj)
        elif len(obj) == 2:
            self._from_tuple(obj)
        elif len(obj) == 3 and isinstance(obj, tuple):
            self._from_tuple2d(obj)
        elif len(obj) == 3:
            self._from_array2d(obj)
        elif len(obj) == 7:
            self._from_array(obj)
        else:
            raise TypeError('Unrecognized object type passed to constructor')

    def _from_header(self, header):
        self.parent_frame = header.frame_id
        self.stamp = header.stamp

    def _from_pose(self, pose):
        p, o = pose.position, pose.orientation
        self.position = (p.x, p.y, p.z)
        self.orientation = (o.x, o.y, o.z, o.w)

    def _from_pose_stamped(self, ps):
        self._from_header(ps.header)
        self._from_pose(ps.pose)

    def _from_transform(self, tf):
        p, o = tf.translation, tf.rotation
        self.position = (p.x, p.y, p.z)
        self.orientation = (o.x, o.y, o.z, o.w)

    def _from_transform_stamped(self, tfs):
        self._from_header(tfs.header)
        self.child_frame = tfs.child_frame_id
        self._from_transform(tfs.transform)

    def _from_tuple(self, tup):
        assert len(tup[0]) == 3
        assert len(tup[1]) == 4
        self.position = tuple(tup[0])
        self.orientation = tuple(tup[1])

    def _from_tuple2d(self, tup):
        self.position = tup[0:2] + (0.0,)
        self.orientation = tr.quaternion_from_euler(0, 0, tup[2])

    def _from_array(self, arr):
        self.position = tuple(arr[:3])
        self.orientation = tuple(arr[3:])

    def _from_array2d(self, arr):
        self._from_tuple2d(tuple(arr))

    def _to_header(self):
        header = Header()
        header.frame_id = self.parent_frame
        header.stamp = self.stamp
        return header

    def to_pose(self):
        pose = Pose()
        p, o = pose.position, pose.orientation
        (p.x, p.y, p.z) = self.position
        (o.x, o.y, o.z, o.w) = self.orientation
        return pose

    def to_pose_stamped(self):
        ps = PoseStamped()
        ps.header = self._to_header()
        ps.pose = self.to_pose()
        return ps

    def to_transform(self):
        tf = Transform()
        p, o = tf.translation, tf.rotation
        (p.x, p.y, p.z) = self.position
        (o.x, o.y, o.z, o.w) = self.orientation
        return tf

    def to_transform_stamped(self):
        tfs = TransformStamped()
        tfs.header = self._to_header()
        tfs.child_frame_id = self.child_frame
        tfs.transform = self.to_transform()
        return tfs

    def to_tuple(self):
        return self.position, self.orientation

    def to_tuple2d(self):
        return self.position[0:2] + (tr.euler_from_quaternion(self.orientation)[2],)

    def to_array(self):
        return np.concatenate(self.to_tuple())

    def to_array2d(self):
        return np.array(self.to_tuple2d())

    def __repr__(self):
        return ' ' + pprint.pformat(self.__dict__)[1:-1]


def to_pose(obj):
    return Converter(obj).to_pose()


def to_pose_stamped(obj):
    return Converter(obj).to_pose_stamped()


def to_transform(obj):
    return Converter(obj).to_transform()


def to_transform_stamped(obj):
    return Converter(obj).to_transform_stamped()


def to_tuple(obj):
    return Converter(obj).to_tuple()

def to_tuple2d(obj):
    return Converter(obj).to_tuple2d()

def to_array(obj):
    return Converter(obj).to_array()

def to_array2d(obj):
    return Converter(obj).to_array2d()
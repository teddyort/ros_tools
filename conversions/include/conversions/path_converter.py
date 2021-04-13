""" path_converter.py
Author: Teddy Ort
path_converter provides methods for converting between nav_msgs.msg.Path objects and numpy array
representations in 2D and 3D
"""

import numpy as np
from nav_msgs.msg import Path

import pose_converter as pc

class PathConverter(object):
    def __init__(self, obj=None):
        # Initialize
        self.poses = None

        if isinstance(obj, Path):
            self._from_path(obj)
            return

        obj = np.asarray(obj)
        if len(obj.shape) > 1 and obj.shape[1] == 7:
            self._from_array(obj)
            return

        if len(obj.shape) > 1 and obj.shape[1] == 3:
            self._from_array2d(obj)
            return

        raise TypeError('Unrecognized object type passed to constructor')

    def _from_path(self, path):
        self.poses = np.array(map(pc.to_array, path.poses))

    def _from_array(self, arr):
        self.poses = arr

    def _from_array2d(self, arr):
        self.poses = np.array(map(pc.to_array, arr))

    def to_path(self):
        path = Path()
        path.poses = [pc.to_pose_stamped(p) for p in self.poses]
        return path

    def to_array(self):
        return self.poses

    def to_array2d(self):
        return np.array(map(pc.to_array2d, self.poses))

    def __repr__(self):
        return "PathConverter\nposes " + str(self.poses)


def to_path(obj):
    return PathConverter(obj).to_path()

def to_array(obj):
    return PathConverter(obj).to_array()

def to_array2d(obj):
    return PathConverter(obj).to_array2d()

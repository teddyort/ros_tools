#!/usr/bin/env python
import unittest, rosunit
import numpy as np
from nav_msgs.msg import OccupancyGrid

from conversions import PyOccupancyGrid

class PyOccupancyGridTester(unittest.TestCase):
    longMessage = True # Gives more descriptive failure messages

    def setUp(self):
        """ Setup some sample data for the tests. """
        # The sample grid will be a 10x10 square with each cell having a width of 10m. We will
        # center the origin at [50,50] so that the grid goes from [-50 50]. Then we will set all
        # values in the left half to 0, and the right to 1.
        grid = np.ones((10,10))
        grid[:,0:5] = 0
        self.pygrid = PyOccupancyGrid(grid, 10, 10, 10, [-50, -50])

    def test_instantiation(self):
        """ Ensure that the object can be instantiated"""
        self.assertIsInstance(self.pygrid, PyOccupancyGrid)

    def test_convert_coords(self):
        """ Create a sample grid and convert a few values"""
        # Position of the origin
        xy = self.pygrid.index_to_coord([0, 0])
        np.testing.assert_allclose(xy, [-50, -50])

        # Lower left corner of the top-right cell
        xy  = self.pygrid.index_to_coord([9, 9])
        np.testing.assert_allclose(xy, [40, 40])

        # The [0.0,0.0] coords are the origin for the [5,5] cell
        rc = self.pygrid.coord_to_index([0.0, 0.0])
        np.testing.assert_allclose(rc, [5, 5])

    def test_is_occupied(self):
        """ Test checking if a cell is occupied """
        # We know the top right cell is occupied because the right half is all 1's
        self.assertTrue(self.pygrid.is_occupied([50,50]))

    def test_is_occupied_vectorized(self):
        """ Lookup a large set of points at once """
        # Choose random points in the area covered by the grid
        N = 100 # number of test points
        np.random.seed(42) # Make randomness in tests repeatable to prevent gray hairs
        xy = np.random.rand(N, 2) * 100 -50
        occupied = self.pygrid.is_occupied(xy)
        np.testing.assert_allclose(occupied, xy[:,0] >= 0 )

    def test_max_cost(self):
        """ Test that setting a max_cost can change the behavior of is_occupied"""

        # Add a higher cost to the far right column
        self.pygrid.grid[:,-1] = 2

        # [25,0] is in the right half, but if we allow max_cost=1 it shouldn't be occupied
        self.assertFalse(self.pygrid.is_occupied([25, 0], max_cost=1),
                         "[25,0] is not occupied since we set a max_cost=1")

        # But [50,0] is in the last column so it should still fail
        self.assertTrue(self.pygrid.is_occupied([50, 0], max_cost=1),
                        "The last column is occupied with a cost of 2 above the max_cost of 1")

    def test_roundtrip(self):
        """ Test converting from a ROS message and back again """
        msg_in = OccupancyGrid()
        msg_in.data = [1, 2, 3, 4]
        msg_in.info.origin.position.x, msg_in.info.origin.position.y = [5, 10]
        msg_in.info.width = 2
        msg_in.info.height = 2
        msg_in.info.resolution = 0.5

        msg_out = PyOccupancyGrid(msg_in).to_msg()

        self.assertEqual(msg_in.info.width, msg_out.info.width)
        self.assertEqual(msg_in.info.height, msg_out.info.height)
        self.assertEqual(msg_in.info.resolution, msg_out.info.resolution)
        self.assertEqual(msg_in.info.origin.position.x, msg_out.info.origin.position.x)
        self.assertEqual(msg_in.info.origin.position.y, msg_out.info.origin.position.y)
        np.testing.assert_allclose(msg_in.data, msg_out.data)





if __name__ == '__main__':
    rosunit.unitrun('conversions', 'py_occupancy_grid_tester', PyOccupancyGridTester)

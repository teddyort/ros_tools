from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
import matplotlib.pyplot as plt

class PyOccupancyGrid(object):
    """ A python representation of an occupancy grid stored as a numpy array """

    def __init__(self, data, width=None, height=None, resolution=None, origin=None):
        """
        Initialize a PyOccupancyGrid. If the type of data is OccupancyGrid, all other parameters
        are ignored. Otherwise, the other parameters are used to build the grid.
        Args:
            data (list) or (nav_msgs.msg.OccupancyGrid): The occupancy data
            width (int): Grid width (number of columns)
            height (int): Grid height (number of rows)
            resolution (float): Grid resolution. Size of each cell in meters
            origin (list) or (geometry_msgs.msg.Pose): The pose of the origin cell [m, m]
        """

        if isinstance(data, OccupancyGrid):
            width = data.info.width
            height = data.info.height
            resolution = data.info.resolution
            origin = data.info.origin
            data = data.data

        if isinstance(origin, Pose):
            origin = [origin.position.x, origin.position.y]

        self.origin = np.array(origin) #TODO add support for rotated grids origin=[x,y,theta]
        self.shape = np.array([height, width])
        self.resolution = resolution
        self.grid = np.array(data, dtype=int).reshape(self.shape)

    def index_to_coord(self, rc):
        """ Convert row column [r, c] to xy coordinates[x, y]

        Args:
            rc: Either a single cell [r,c] or an array of cells with shape (N,2)
        """
        rc = np.asarray(rc)
        return np.flip(self.resolution * rc, -1) + self.origin

    def coord_to_index(self, xy):
        """ Convert xy coordinates [x, y] to row column [r, c]
        Args:
            xy (arraylike): Either a single coordinate [x,y] or array with shape (N,2)
        """
        xy = np.asarray(xy)
        rc = np.flip(((xy - self.origin) // self.resolution).astype(int), -1)
        return rc.clip((0, 0), self.shape - 1) # Clip to array size

    def is_occupied(self, xy, max_cost=0):
        """ Returns a boolean of the same shape as xy True where the corresponding cell > max_cost.
        Args:
            xy: Either a single coordinate [x,y] or array with shape (N,2)
            max_cost (float): The maximum cost to consider a cell unoccupied. default: 0
        """
        return self.get_grid_value(xy) > max_cost

    def get_grid_value(self, xy):
        """ Return the grid value of the py_occupancy grid.
        Args:
            xy: Either a single coordinate [x,y] or array with shape (N,2)
        """
        rc = self.coord_to_index(xy)
        return self.grid[rc.T[0], rc.T[1]]

    def to_msg(self):
        """ Returns the grid as a ros nav_msgs.msg.OccupancyGrid. """
        msg = OccupancyGrid()
        msg.data = self.grid.flatten().tolist()
        msg.info.height, msg.info.width = self.shape
        msg.info.resolution = self.resolution
        msg.info.origin.position.x, msg.info.origin.position.y = self.origin
        return msg

    def get_extent(self):
        """ Returns the extent of the grid in cartesian coordinates (left, right, bottom, top)."""
        top_right = self.resolution*(np.flipud(self.shape)) + self.origin
        return self.origin[0], top_right[0], self.origin[1], top_right[1]

    def plot(self):
        """ Plots the grid as an image and returns a handle to the image axes """
        return plt.imshow(self.grid, extent=self.get_extent(), origin='lower')
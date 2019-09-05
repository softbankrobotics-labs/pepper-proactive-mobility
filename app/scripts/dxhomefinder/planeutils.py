"""
Library for 2D manipulation
"""
# pylint: disable=E1101

__version__ = "0.1.0"

__copyright__ = "Copyright 2017, Softbank Robotics"
__author__ = 'mcaniot'
__email__ = 'mcaniot@softbankrobotics.com'

# Basic libs
import cmath
import math
import numpy as np

###########################################
#            Useful Function              #
###########################################

def coords_to_pos(x_coord, y_coord):
    "A pos = imaginary numbers"
    return x_coord + y_coord * 1j

def pos_to_coords(pos):
    "Transform pose to x, y coordinate"
    r_value, p_value = cmath.polar(pos)
    x_coord = r_value*np.cos(p_value)
    y_coord = r_value*np.sin(p_value)
    return x_coord, y_coord

def unit_vector(angle_in_rad):
    "Return unit vertor of angle"
    return math.e ** (1j * angle_in_rad)

def get_pos_theta(pos):
    "Get angle theta of a position"
    value = cmath.polar(pos)
    return value[1]

def get_pos_dist(pos):
    "Get distance of position"
    return abs(pos)

def get_pos_to_pos_dist(pos1, pos2):
    "Get distance between to position"
    return abs(pos1 - pos2)

class Pose(object):
    """
    Object Pose for 2D manipulation
    """
    def __init__(self, **kwargs):
        # Arguments are x, y, theta, pos and orientation.
        # But you can construct this class with only x,y and theta or
        # pos and orientation.

        # internal variables
        if "x_coord" in kwargs:
            self.x_coord = kwargs["x_coord"]
        else:
            self.x_coord = None

        if "y_coord" in kwargs:
            self.y_coord = kwargs["y_coord"]
        else:
            self.y_coord = None

        if "theta" in kwargs:
            self.theta = kwargs["theta"]
        else:
            self.theta = None

        if "pos" in kwargs:
            self.pos = kwargs["pos"]
        elif self.x_coord != None and self.y_coord != None:
            self.pos = coords_to_pos(self.x_coord, self.y_coord)
        else:
            self.pos = None

        if "orientation" in kwargs:
            self.orientation = kwargs["orientation"]
        elif self.theta != None:
            self.orientation = unit_vector(self.theta)
        else:
            self.orientation = None

    ###########################################
    #            Public Method                #
    ###########################################

    def get_relative(self, pose):
        "Expects a pos in same referential as myself"
        relative_pos = (pose.pos - self.pos) / self.orientation
        relative_orientation = pose.orientation / self.orientation
        return Pose(pos=relative_pos, orientation=relative_orientation)

    def get_relative_pos(self, pos):
        "Get relative pos"
        return (pos - self.pos) / self.orientation

    def get_coord(self):
        if self.x_coord != None and self.y_coord != None and self.theta != None:
            return self.x_coord, self.y_coord, self.theta
        elif self.pos != None:
            x_robot, y_robot = pos_to_coords(self.pos)
            theta_robot = get_pos_theta(self.pos)
            return float(x_robot), float(y_robot), float(theta_robot)
        else:
            return None, None, None


    ###########################################
    #            Private Method               #
    ###########################################

    def __eq__(self, pose):
        return self.pos == pose.pos and self.orientation == pose.orientation

    def __str__(self):
        return "pos = %s, orient = %s" % (self.pos, self.orientation)

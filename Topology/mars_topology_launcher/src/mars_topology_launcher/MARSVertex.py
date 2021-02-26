#!/usr/bin/env python
"""
    Fraunhofer IML
    Department Automation and Embedded Systems
    Tabsize          : 4
    Charset          : UTF-8
"""
__author__ = "Dennis Luensch"
__maintainer__ = "Dennis Luensch"
__email__ = "dennis.luensch@iml.fraunhofer.de"

from MARSEntity import MARSEntity
from MARSEntityTypes import MARSEntityTypes

import rospy

import os
import math
from enum import Enum

DEFAULT_FOOTPRINT_RADIUS = 1.0
DEFAULT_FOOTPRINT_RESOLUTION = 6


class MARSFootprintType(Enum):

    MARS_FOOTPRINT_TYPE_UNKNOWN = 0
    MARS_FOOTPRINT_TYPE_CIRCLE = 1
    MARS_FOOTPRINT_TYPE_SQUARE = 2


class MARSVertex(MARSEntity):
    def __init__(self, id, x_position=None, y_position=None,
                 type=MARSEntityTypes.TOPOLOGY_VERTEX_TYPE_WAYPOINT,
                 footprint_resolution=DEFAULT_FOOTPRINT_RESOLUTION,
                 footprint_radius=DEFAULT_FOOTPRINT_RADIUS,
                 footprint_type=MARSFootprintType.MARS_FOOTPRINT_TYPE_SQUARE,
                 calc_footprint=False, max_linear_velocity=None,
                 max_angular_velocity=None,
                 max_linear_acceleration=None,
                 max_angular_acceleration=None):
        MARSEntity.__init__(self, id, type=type,
                            max_linear_velocity=max_linear_velocity,
                            max_angular_velocity=max_angular_velocity,
                            max_linear_acceleration=max_linear_acceleration,
                            max_angular_acceleration=max_angular_acceleration)

        self.__incoming_edge_ids = []
        self.__outgoing_edges_ids = []

        if (x_position is not None) or (y_position is not None):
            self.set_position(x_position, y_position)
        else:
            self._x_position = 0.0
            self._y_position = 0.0

        if (calc_footprint):
            if (footprint_type == MARSFootprintType.MARS_FOOTPRINT_TYPE_SQUARE):
                self.calc_square_footprint(footprint_radius)
            elif (footprint_type == MARSFootprintType.MARS_FOOTPRINT_TYPE_CIRCLE):
                self.calc_circle_footprint(footprint_radius, footprint_resolution)
            else:
                rospy.logwarn(
                    "[MARSVertex][MARSVertex] Unknown footprint type for creating footprint was given. Continue with calculating circle footprint.")
                self.calc_circle_footprint(footprint_radius, footprint_resolution)

    def set_position(self, x_position, y_position):
        self._x_position = x_position
        self._y_position = y_position

    def add_incoming_edge_id(self, __incoming_edge_ids):
        self.__incoming_edge_ids.append(__incoming_edge_ids)

    def add_outgoing_edge_id(self, outgoing_edge):
        self.__outgoing_edges_ids.append(outgoing_edge)

    def get_incoming_edge_ids(self):
        return self.__incoming_edge_ids

    def get_outgoing_edge_ids(self):
        return self.__outgoing_edges_ids

    def calc_circle_footprint(
            self, footprint_radius, footprint_resolution, origin_x=None, origin_y=None):
        self.__clear_footprint()

        if (origin_x is None or origin_y is None):
            origin_x = self._x_position
            origin_y = self._y_position

        angle = 2 * math.pi / footprint_resolution

        for i in range(0, footprint_resolution):
            x_cord = (math.cos(i * angle) * footprint_radius)
            y_cord = (math.sin(i * angle) * footprint_radius)
            self._footprint_x.append(origin_x + x_cord)
            self._footprint_y.append(origin_y + y_cord)

    def calc_square_footprint(self, footprint_radius, origin_x=None, origin_y=None):
        self.__clear_footprint()

        if (origin_x is None or origin_y is None):
            origin_x = self._x_position
            origin_y = self._y_position

        #          ^
        #          |
        #   II     |      I
        #          |
        # ---------|---------->
        #          |
        #   III    |      IV
        #          |
        #          |

        # set footprint for 1st quadrant
        self._footprint_x.append(origin_x + footprint_radius)
        self._footprint_y.append(origin_y + footprint_radius)
        # set footprint for 2nd quadrant
        self._footprint_x.append(origin_x - footprint_radius)
        self._footprint_y.append(origin_y + footprint_radius)
        # set footprint for 3rd quadrant
        self._footprint_x.append(origin_x - footprint_radius)
        self._footprint_y.append(origin_y - footprint_radius)
        # set footprint for 4th quadrant
        self._footprint_x.append(origin_x + footprint_radius)
        self._footprint_y.append(origin_y - footprint_radius)

    def __clear_footprint(self):
        # Python 2.7
        self._footprint_x = []
        self._footprint_y = []

        # Python 3.3
        # self._footprint_x.clear()
        # self._footprint_y.clear()

    def __str__(self):
        return ("Vertex: " + str(self.get_id()) + os.linesep
                + "position: (" + str(self.get_x_position())
                + ", " + str(self.get_y_position()) + ")" + os.linesep
                + "type: " + str(self.get_type()) + os.linesep
                + "incoming edges: " +
                str(self.__incoming_edge_ids) + os.linesep
                + "outgoing edges: " +
                str(self.__outgoing_edges_ids) + os.linesep
                + "footprintX: " + str(self._footprint_x) + os.linesep
                + "footprintY: " + str(self._footprint_y) + os.linesep
                + "----------------------------------")

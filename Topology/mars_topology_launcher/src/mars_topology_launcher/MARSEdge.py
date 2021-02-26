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
from MARSPath import MARSPath
from MARSEntityTypes import MARSEntityTypes

import os


class MARSEdge(MARSEntity):
    def __init__(self, id,  x_position=0.0, y_position=0.0,
                 length=None, type=MARSEntityTypes.TOPOLOGY_EDGE_TYPE_EDGE,
                 max_linear_velocity=None,
                 max_angular_velocity=None,
                 max_linear_acceleration=None,
                 max_angular_acceleration=None):
        MARSEntity.__init__(self, id, type=type,
                            max_linear_velocity=max_linear_velocity,
                            max_angular_velocity=max_angular_velocity,
                            max_linear_acceleration=max_linear_acceleration,
                            max_angular_acceleration=max_angular_acceleration)
        self.__length = length

        if (x_position is not None) or (y_position is not None):
            self.set_position(x_position, y_position)

        self.__paths = []

    def set__length(self, length):
        """Sets the length of the MARSEdge.

        Sets the length of the MARSEdge. The lenght is in meter [m]!

        Args:
            length: Length of the edge in meter [m]

        Returns:

        Raises:
        """
        self.__length = length

    def get_length(self):
        """Returns the length of the MARSEdge.

        Returns the length of the  MARSEdge. The length is in meter!

        Args:

        Returns:
            Returns the length of the MARSEdge in meter [m].

        Raises:
        """
        return self.__length

    def add_path(self, origin_vertex_id, dst_vertex_id, max_velocity):
        self.__paths.append(
            MARSPath(origin_vertex_id, dst_vertex_id, max_velocity))

    def get_paths(self):
        return self.__paths

    def __str__(self):
        return ("Edge: " + str(self.get_id()) + os.linesep
                + "position: (" + str(self.get_x_position())
                + ", " + str(self.get_y_position()) + ")" + os.linesep
                + "type: " + str(self.get_type()) + os.linesep
                + "length: " + str(self.__length) + os.linesep
                + "path(s): " + str(self.__paths) + os.linesep
                + "----------------------------------")

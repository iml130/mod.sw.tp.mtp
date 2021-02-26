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

from abc import ABCMeta, abstractmethod
import xml.etree.ElementTree

from MARSVertex import MARSVertex, MARSFootprintType, DEFAULT_FOOTPRINT_RESOLUTION
from MARSEdge import MARSEdge
from mars_common.Id import Id, IdType

import rospy


class TopologyParser():
    """Abstract class for parsing topologies.

    Abstract class for parsing topologies.

    Attributes:
        __id: Current id for the topology entity that will be created.
                After creating an entity the id must be increased.
        _mars_vertices: Contains all mars vertices which has to be started
                            in a dictionary.
                            key = id of the vertex, value = MARSVertex
        _mars_edges: Contains all mars edges which has to be started
                        in a dictionary.
                        key = id of the edge, value = MARSEdge
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        self.__id = 0
        self._mars_vertices = dict()
        self._mars_edges = dict()

    @abstractmethod
    def parse_file(self, file_path):
        """Reads a topology file an creates mars edge and vertex entities.

        Reads a topology file an creates mars edge and vertex entities.
        This entities can be used to start topology nodes (edges and vertices).

        Args:
            file_path: path to the file on the system as a string.

        Returns:
            Returns "True" if the file was successfully opened and parsed!

        Raises:
        """
        pass

    def get_mars_topology_vertices(self):
        """Return all MARSVertex objects.

        Returns all MARSVertex objects in a dictionary.

        Args:

        Returns:
            Returns a dictionary with all MARSVertex objects.

        Raises:
        """
        return self._mars_vertices

    def get_mars_topology_edges(self):
        """Return all MARSEdge objects.

        Returns all MARSEdge objects in a dictionary. 

        Args:

        Returns:
            Returns a dictionary with all MARSEdge objects.

        Raises:
        """
        return self._mars_edges

    def _create_mars_vertex(
            self, vertex_name, x_position, y_position, footprint_type, footprint_radius,
            footprint_resolution=DEFAULT_FOOTPRINT_RESOLUTION, uuid=None,
            uuid_type=IdType.ID_TYPE_STRING_UUID,
            footprint_x=None, footprint_y=None):
        """Creates an object of type MARSVertex.

        Creates an object of type MARSVertex and sets an unique id!

        Args:
            vertex_name: Name of the vertex.
            x_position: X-Position of the vertex.
            y_position: Y-Position of the vertex.
            uuid: A string based uuid or name.
            uuid_tpye: Type of the given uuid.
        Returns:
            Return the created MARSVertex object.

        Raises:
        """
        if uuid is not None:
            mars_vertex = MARSVertex(Id(uuid, uuid_type, description=vertex_name))
        else:
            mars_vertex = MARSVertex(
                Id(self.__id, IdType.ID_TYPE_STRING_NAME, description=vertex_name))
        mars_vertex.set_name(vertex_name)
        mars_vertex.set_position(x_position, y_position)

        if footprint_x and footprint_y:
            mars_vertex.add_footprint(footprint_x, footprint_y)
        else:
            if (footprint_type == MARSFootprintType.MARS_FOOTPRINT_TYPE_SQUARE):
                mars_vertex.calc_square_footprint(footprint_radius)
            elif (footprint_type == MARSFootprintType.MARS_FOOTPRINT_TYPE_CIRCLE):
                mars_vertex.calc_circle_footprint(footprint_radius, footprint_resolution)
            else:
                rospy.logwarn(
                    "[TopologyParser][_create_mars_vertex] Unknown footprint type for creating footprint was given. Continue with calculating circle footprint.")
                mars_vertex.calc_circle_footprint(footprint_radius, footprint_resolution)

        self.__id = self.__id + 1

        return mars_vertex

    def _create_mars_edge(self, edge_name, length, uuid=None,
                          footprint_x=None, footprint_y=None):
        """Creates an object of type MARSVertex.

        Creates an object of type MARSVertex and sets an unique id!

        Args:
            edge_name: Name of the edge.
            length: Length of the edge in meter (float).
            max_velocity: Maximum allowed velocity on the edge on m/s (float).
            uuid: A string based uuid (optional)
        Returns:
            Return the created MARSVertex object.

        Raises:
        """
        if uuid is not None:
            mars_edge = MARSEdge(
                Id(uuid, IdType.ID_TYPE_STRING_UUID), length=length)
        else:
            mars_edge = MARSEdge(
                Id(self.__id, IdType.ID_TYPE_STRING_NAME), length=length)
        mars_edge.set_name(edge_name)

        if footprint_x and footprint_y:
            mars_edge.add_footprint(footprint_x, footprint_y)

        self.__id = self.__id + 1

        return mars_edge

    def print_parsed_topology_ros_debug(self):
        self.__print_entity(self._mars_vertices)
        self.__print_entity(self._mars_edges)

    def __print_entity(self, entity_collection):
        for entity_name in entity_collection.values():
            rospy.logdebug(str(entity_name))

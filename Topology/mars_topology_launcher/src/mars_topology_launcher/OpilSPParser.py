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

from TopologyParser import TopologyParser
from MARSEdge import MARSEdge
from MARSVertex import MARSVertex, MARSFootprintType, DEFAULT_FOOTPRINT_RADIUS, DEFAULT_FOOTPRINT_RESOLUTION
from TopologyException import TopologyException

# opil.sw.sp import
from maptogridmap.msg import Graph, Edge, Vertex

import rospy
import os
import math

# Default velocity 1 m / s (meter per second)
_OPIL_DEFAULT_EDGE_VELOCITY = 1.0

_EDGE_TYPE_OUTGOING = "ET_OUTGOING"
_EDGE_TYPE_INCOMING = "ET_INCOMING"


class OpilSPParser(TopologyParser):
    def __init__(self, footprint_type=MARSFootprintType.MARS_FOOTPRINT_TYPE_SQUARE,
                 footprint_radius=DEFAULT_FOOTPRINT_RADIUS,
                 footprint_resolution=DEFAULT_FOOTPRINT_RESOLUTION):

        TopologyParser.__init__(self)

        self.__footprint_type = footprint_type
        self.__footprint_radius = footprint_radius
        self.__footprint_resolution = footprint_resolution

    def parse_file(self, topic_name):
        """Reads a topology file an creates mars edge and vertex entities.

        Reads a topology from a topic and creates mars edge and vertex entities.
        This entities can be used to start topology nodes (edges and vertices).

        Args:
            topic_name: name of the topic.
        Returns:
            Returns "True" if topology was parsed and created!

        Raises:
        """

        parsed_successfully = True

        rospy.logdebug("[OpilSPParser][parse_file] Waiting for topology on topic: " +
                       str(topic_name))

        data = rospy.wait_for_message(topic_name, Graph)

        self._mars_vertices = self.__create_mars_vertices(data.vertices)

        self._mars_edges = self.__create_mars_edges(data.edges)

        return parsed_successfully

    def __create_mars_vertices(self, msg_vertices):
        """Parses all vertices from the OPIL (S&P) Graph msg.

        Parses all vertices from the OPIL (S&P) Graph msg and creates individual
        objects with needed attributes.

        Args:
            msg_vertices: Vertex array of the graph msg.

        Returns:
            Returns a list of all parsed point objects.

        Raises:
        """

        mars_vertices = dict()

        for msg_vertex in msg_vertices:
            # footprint_x = []
            # footprint_y = []

            # Create base node from msg
            mars_vertex = self._create_mars_vertex(
                msg_vertex.name, msg_vertex.x, msg_vertex.y, self.__footprint_type,
                self.__footprint_radius,
                footprint_resolution=self.__footprint_resolution, uuid=msg_vertex.uuid)

            # INFO: Footprint will be remove in net version
            # for point in msg_vertex.footprint:
            #     footprint_x.append(point.x - msg_vertex.x)
            #     footprint_y.append(point.y - msg_vertex.y)
            # mars_vertex.add_footprint(footprint_x, footprint_y)

            mars_vertices[msg_vertex.uuid] = mars_vertex

        return mars_vertices

    def __create_mars_edges(self, msg_edges):
        """Parses all edges from the OPIL (S&P) Graph msg.

        Parses all edges from the OPIL (S&P) Graph msg and creates individual
        objects with needed attributes. Also relates edges and vertices. Before 
        you call this method, the OPIL vertices (__create_mars_vertices) must 
        be parsed and self._mars_vertices must be set!

        Args:
            msg_edges: Edge array of the graph msg.

        Returns:
            Returns a list of all parsed egde objects.

        Raises:
        """
        mars_edges = dict()

        if len(self._mars_vertices):
            for msg_edge in msg_edges:

                # calculate the middle of the edge based on the position of the two vertices.
                mars_edge_length = math.sqrt(
                    math.pow((self._mars_vertices[msg_edge.uuid_src].get_x_position(
                    ) - self._mars_vertices[msg_edge.uuid_dest].get_x_position()), 2) +
                    math.pow((self._mars_vertices[msg_edge.uuid_src].get_y_position(
                    ) - self._mars_vertices[msg_edge.uuid_dest].get_y_position()), 2))

                try:
                    # check existens for opposite direction of the edge
                    mars_edge = self.__check_for_edge_existence(
                        mars_edges, self._mars_vertices[msg_edge.uuid_src].get_id(),
                        self._mars_vertices[msg_edge.uuid_dest].get_id())
                    if mars_edge is None:
                        mars_edge = self._create_mars_edge(
                            msg_edge.name, mars_edge_length, msg_edge.uuid)
                        mars_edge.set_position(
                            ((self._mars_vertices[msg_edge.uuid_src].get_x_position(
                            ) + self._mars_vertices[msg_edge.uuid_dest].get_x_position()) / 2.0),
                            ((self._mars_vertices[msg_edge.uuid_src].get_y_position(
                            ) + self._mars_vertices[msg_edge.uuid_dest].get_y_position()) / 2.0))
                        mars_edges[msg_edge.uuid] = mars_edge

                        # Adding bidirectional edge
                        mars_edge.add_path(
                            self._mars_vertices[msg_edge.uuid_src].get_id(),
                            self._mars_vertices[msg_edge.uuid_dest].get_id(),
                            _OPIL_DEFAULT_EDGE_VELOCITY)
                        mars_edge.add_path(
                            self._mars_vertices[msg_edge.uuid_dest].get_id(),
                            self._mars_vertices[msg_edge.uuid_src].get_id(),
                            _OPIL_DEFAULT_EDGE_VELOCITY)

                        # Add edge to vertex 1 as in- and outgoing
                        self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                     msg_edge.uuid_dest,
                                                     _EDGE_TYPE_INCOMING)
                        self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                     msg_edge.uuid_src,
                                                     _EDGE_TYPE_OUTGOING)

                        # Add edge to vertex 2 as in- and outgoing
                        self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                     msg_edge.uuid_src,
                                                     _EDGE_TYPE_INCOMING)
                        self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                     msg_edge.uuid_dest,
                                                     _EDGE_TYPE_OUTGOING)
                    else:
                        rospy.logwarn("[OpilSPParser][__create_mars_edges]"
                                      " Existing edge found, skipping edge!")
                except TopologyException as err:
                    rospy.logwarn(str(err))
        else:
            rospy.logerr("[OpilSPParser][__create_mars_edges]"
                         " No vertices parsed!")

        return mars_edges

    # def __create_mars_edges(self, msg_edges):
    #     """Parses all edges from the OPIL (S&P) Graph msg.

    #     Parses all edges from the OPIL (S&P) Graph msg and creates individual
    #     objects with needed attributes. Also relates edges and vertices. Before
    #     you call this method, the OPIL vertices (__create_mars_vertices) must
    #     be parsed and self._mars_vertices must be set!

    #     Args:
    #         msg_edges: Edge array of the graph msg.

    #     Returns:
    #         Returns a list of all parsed egde objects.

    #     Raises:
    #     """

    #     mars_edges = dict()

    #     if len(self._mars_vertices):
    #         for msg_edge in msg_edges:

    #             # calculate the middle of the edge based on the position of the two vertices.
    #             mars_edge_length = math.sqrt(
    #                 math.pow((self._mars_vertices[msg_edge.uuid_src].get_x_position(
    #                 ) - self._mars_vertices[msg_edge.uuid_dest].get_x_position()), 2) +
    #                 math.pow((self._mars_vertices[msg_edge.uuid_src].get_y_position(
    #                 ) - self._mars_vertices[msg_edge.uuid_dest].get_y_position()), 2))

    #             try:
    #                 # check existens for opposite direction of the edge
    #                 mars_edge = self.__check_for_edge_existence(
    #                     mars_edges, self._mars_vertices[msg_edge.uuid_src].get_id(),
    #                     self._mars_vertices[msg_edge.uuid_dest].get_id())
    #                 if mars_edge is None:
    #                     mars_edge = self._create_mars_edge(
    #                         msg_edge.name, mars_edge_length, msg_edge.uuid)
    #                     mars_edge.set_position(
    #                         ((self._mars_vertices[msg_edge.uuid_src].get_x_position(
    #                         ) + self._mars_vertices[msg_edge.uuid_dest].get_x_position()) / 2.0),
    #                         ((self._mars_vertices[msg_edge.uuid_src].get_y_position(
    #                         ) + self._mars_vertices[msg_edge.uuid_dest].get_y_position()) / 2.0))
    #                     mars_edges[msg_edge.uuid] = mars_edge

    #                 mars_edge.add_path(
    #                     self._mars_vertices[msg_edge.uuid_src].get_id(),
    #                     self._mars_vertices[msg_edge.uuid_dest].get_id(),
    #                     _OPIL_DEFAULT_EDGE_VELOCITY)

    #                 # dst_point_id == current_point_id: edge_id --> dst_point (incoming edge)
    #                 self.__add_edge_id_to_vertex(mars_edge.get_id(),
    #                                              msg_edge.uuid_dest,
    #                                              _EDGE_TYPE_INCOMING)
    #                 # src_point_id == current_point_id: src_point --> edge_id  (outgoing edge)
    #                 self.__add_edge_id_to_vertex(mars_edge.get_id(),
    #                                              msg_edge.uuid_src,
    #                                              _EDGE_TYPE_OUTGOING)
    #             except TopologyException as err:
    #                 rospy.logwarn(str(err))
    #     else:
    #         rospy.logerr("[OpilSPParser][__create_mars_edges]"
    #                      " No vertices parsed!")

    #     return mars_edges

    def __add_edge_id_to_vertex(self, edge_id, vertex_name, edge_type):
        if vertex_name in self._mars_vertices:
            if edge_type == _EDGE_TYPE_INCOMING:
                self._mars_vertices[vertex_name].add_incoming_edge_id(edge_id)
            elif edge_type == _EDGE_TYPE_OUTGOING:
                self._mars_vertices[vertex_name].add_outgoing_edge_id(edge_id)
            else:
                rospy.logerr("[OpilSPParser][__add_edge_id_to_vertex]"
                             " Unsupported edge type given: " + edge_type)
        else:
            rospy.logerr(
                "[OpilSPParser][__add_edge_id_to_vertex]"
                " Can't add edge (" + edge_id + ") to vertex (" + vertex_name +
                "). Vertex doesn't exists!")

    def __check_for_edge_existence(self, mars_edges,
                                   origin_vertex_id, dst_vertex_id):
        """Checks whether an edge between the two vertex in the opposite
        direction exits.

        Checks whether an edge between the two vertex in the opposite
        direction exits. If an edge in the same direction exists, an exception
        will be thrown

        Args:
            mars_edges: Array which consits of all created edges.
            origin_vertex_id: id of the origin vertex of the edge
            dst_vertex_id: id of the destination vertex of the edge

        Returns:
            Returns:
                - None if no opposite direction in the given edges can be found.
                - An edge will be returned if the opposite path in the 
                    edge exists.

        Raises:
            A TopologyException is raised if an edge consists of the same path
            as given with 'origin_vertex_id' and 'dst_vertex_id'!
        """
        existing_edge = None

        for edge in mars_edges.values():
            tmp_paths = edge.get_paths()

            if existing_edge is not None:
                break

            for path in tmp_paths:
                if (path.get_origin_vertex_id() == dst_vertex_id) \
                        and (path.get_dst_vertex_id() == origin_vertex_id):
                    existing_edge = edge
                elif (path.get_dst_vertex_id() == dst_vertex_id) \
                        and (path.get_origin_vertex_id() == origin_vertex_id):
                    raise TopologyException((
                        "Duplicated path found. Path from point "
                        + str(path.get_origin_vertex_id().get_description())
                        + " (" + str(path.get_dst_vertex_id().get_id())
                        + ") to "
                        + str(path.get_origin_vertex_id().get_description())
                        + " (" + str(path.get_dst_vertex_id().get_id())
                        + ") already exists in edge "
                        + str(edge.get_id().get_description())
                        + " (" + str(edge.get_id().get_id()) + ")!"))

        return existing_edge

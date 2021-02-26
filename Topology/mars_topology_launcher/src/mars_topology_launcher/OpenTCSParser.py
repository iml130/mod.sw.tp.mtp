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

import xml.etree.ElementTree as ET
from TopologyParser import TopologyParser
from MARSEdge import MARSEdge
from MARSVertex import MARSVertex, MARSFootprintType, DEFAULT_FOOTPRINT_RADIUS, DEFAULT_FOOTPRINT_RESOLUTION
from TopologyException import TopologyException
from mars_common.Id import IdType

import rospy
import os

# OpenTCS point constants
_OPENTCS_POINT_TAG_POINT = "point"
_OPENTCS_POINT_ATTRIBUTE_NAME = "name"
_OPENTCS_POINT_ATTRIBUTE_X_POSITION = "xPosition"
_OPENTCS_POINT_ATTRIBUTE_Y_POSITION = "yPosition"

# OpenTCS path constants
_OPENTCS_PATH_TAG_PATH = "path"
_OPENTCS_PATH_ATTRIBUTE_NAME = "name"
_OPENTCS_PATH_ATTRIBUTE_SRC_POINT = "sourcePoint"
_OPENTCS_PATH_ATTRIBUTE_DST_POINT = "destinationPoint"
_OPENTCS_PATH_ATTRIBUTE_LENGTH = "length"
_OPENTCS_PATH_ATTRIBUTE_MAX_VELOCITY = "maxVelocity"

_EDGE_TYPE_OUTGOING = "ET_OUTGOING"
_EDGE_TYPE_INCOMING = "ET_INCOMING"


class OpenTCSParser(TopologyParser):
    def __init__(self, footprint_type=MARSFootprintType.MARS_FOOTPRINT_TYPE_SQUARE,
                 footprint_radius=DEFAULT_FOOTPRINT_RADIUS,
                 footprint_resolution=DEFAULT_FOOTPRINT_RESOLUTION):

        TopologyParser.__init__(self)

        self.__footprint_type = footprint_type
        self.__footprint_radius = footprint_radius
        self.__footprint_resolution = footprint_resolution

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
        parsed_successfully = True

        rospy.logdebug("[OpenTCSParser][parse_file] Parsing file: " +
                       str(file_path))

        tree = ET.parse(file_path)

        if tree:
            root = tree.getroot()

            self._mars_vertices = self.__parse_opentcs_points(root)
            self._mars_edges = self.__parse_opentcs_paths(root)

            self.print_parsed_topology_ros_debug()
        else:
            parsed_successfully = False
            rospy.logerr("[OpenTCSParser][parse_file] Could not parse file!")

        return parsed_successfully

    def __parse_opentcs_points(self, root):
        """Parses all point objects in the OpenTCS file.

        Parses all point objects in the OpenTCS file and creates individual
        objects with needed attributes.

        Args:
            root: root element of the parsed OpenTCS xml tree.

        Returns:
            Returns a list of all parsed point objects.

        Raises:
        """
        mars_vertices = dict()

        for opentcs_point in root.findall(_OPENTCS_POINT_TAG_POINT):

            point_name = opentcs_point.get(
                _OPENTCS_POINT_ATTRIBUTE_NAME)
            point_x_pos = opentcs_point.get(
                _OPENTCS_POINT_ATTRIBUTE_X_POSITION)
            point_y_pos = opentcs_point.get(
                _OPENTCS_POINT_ATTRIBUTE_Y_POSITION)

            # convert position from millimeter to meter [mm -> m]
            mars_vertex_x_pos = float(point_x_pos) / 1000.0
            mars_vertex_y_pos = float(point_y_pos) / 1000.0

            mars_vertex = self._create_mars_vertex(
                point_name, mars_vertex_x_pos, mars_vertex_y_pos, self.__footprint_type, self.
                __footprint_radius, footprint_resolution=self.__footprint_resolution, uuid=str(point_name), uuid_type=IdType.ID_TYPE_STRING_NAME)

            mars_vertices[point_name] = mars_vertex

        return mars_vertices

    def __parse_opentcs_paths(self, root):
        """Parses all path objects in the OpenTCS file.

        Parses all path objects in the OpenTCS file and creates individual
        objects with needed attributes. Also relates edges and vertices. Before 
        you call this method, the open tcs points (__parse_opentcs_points) must 
        be parsed and self._mars_vertices must be set!

        Args:
            root: root element of the parsed OpenTCS xml tree.

        Returns:
            Returns a list of all parsed path objects.

        Raises:
        """
        mars_edges = dict()

        # check whether points of the topology were successfully parsed
        if len(self._mars_vertices):
            for opentcs_path in root.findall(_OPENTCS_PATH_TAG_PATH):

                path_name = opentcs_path.get(
                    _OPENTCS_PATH_ATTRIBUTE_NAME)
                path_length = opentcs_path.get(
                    _OPENTCS_PATH_ATTRIBUTE_LENGTH)
                path_max_velocity = opentcs_path.get(
                    _OPENTCS_PATH_ATTRIBUTE_MAX_VELOCITY)

                # convert position from millimeter to meter [mm -> m]
                mars_edge_length = float(path_length) / 1000.0
                # convert position from millimeter to meter [mm/s -> m/s]
                mars_edge_max_velocity = float(path_max_velocity) / 1000.0

                path_src_point_name = opentcs_path.get(
                    _OPENTCS_PATH_ATTRIBUTE_SRC_POINT)
                path_dst_point_name = opentcs_path.get(
                    _OPENTCS_PATH_ATTRIBUTE_DST_POINT)

                try:
                    mars_edge = self.__check_for_edge_existence(
                        mars_edges, self._mars_vertices[path_src_point_name].get_id(),
                        self._mars_vertices[path_dst_point_name].get_id())
                    if mars_edge is None:
                        mars_edge = self._create_mars_edge(
                            path_name, mars_edge_length)
                        mars_edge.set_position(
                            ((self._mars_vertices[path_src_point_name].get_x_position(
                            ) + self._mars_vertices[path_dst_point_name].get_x_position()) / 2.0),
                            ((self._mars_vertices[path_src_point_name].get_y_position(
                            ) + self._mars_vertices[path_dst_point_name].get_y_position()) / 2.0))
                        mars_edges[path_name] = mars_edge

                    mars_edge.add_path(
                        self._mars_vertices[path_src_point_name].get_id(),
                        self._mars_vertices[path_dst_point_name].get_id(),
                        mars_edge_max_velocity)

                    # dst_point_id == current_point_id: edge_id --> dst_point (incoming edge)
                    self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                 path_dst_point_name,
                                                 _EDGE_TYPE_INCOMING)
                    # src_point_id == current_point_id: src_point --> edge_id  (outgoing edge)
                    self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                 path_src_point_name,
                                                 _EDGE_TYPE_OUTGOING)
                except TopologyException as err:
                    rospy.logwarn(str(err))
        else:
            rospy.logerr("[OpenTCSParser][__parse_opentcs_paths]"
                         " No opentcs points parsed!")

        return mars_edges

    def __add_edge_id_to_vertex(self, edge_id, vertex_name, edge_type):
        if vertex_name in self._mars_vertices:
            if edge_type == _EDGE_TYPE_INCOMING:
                self._mars_vertices[vertex_name].add_incoming_edge_id(edge_id)
            elif edge_type == _EDGE_TYPE_OUTGOING:
                self._mars_vertices[vertex_name].add_outgoing_edge_id(edge_id)
            else:
                rospy.logerr("[OpenTCSParser][__add_edge_id_to_vertex]"
                             " Unsupported edge type given: " + edge_type)

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

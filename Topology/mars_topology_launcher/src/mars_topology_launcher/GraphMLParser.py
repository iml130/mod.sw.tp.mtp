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

import rospy
import os
import math

# GraphML key constants
_GRAPHML_KEY_TAG_KEY = "key"
_GRAPHML_KEY_ATTRIBUTE_ID = "id"
_GRAPHML_KEY_FOR = "for"
_GRAPHML_KEY_ATTRIBUTE_NAME = "attr.name"

# GRAPHML possible for tags
_GRAPHML_KEY_TAG_FOR_NODE = "node"
_GRAPHML_KEY_TAG_FOR_EDGE = "edge"

# GraphML data entries (node and edge)
_GRAPHML_DATA_TAG_DATA = "data"
_GRAPHML_DATA_ATTRIBUTE_KEY = "key"

# GraphML ambiguous constants for vertices and edges
_GRAPHML_ATTRIBUTE_NAME = "name"
_GRAPHML_ATTRIBUTE_ID = "uuid"
_GRAPHML_ATTRIBUTE_POLYGON_POINTS = "polygon-points"

# GraphML node constants
_GRAPHML_NODE_TAG_NODE = "node"
_GRAPHML_NODE_ATTRIBUTE_NAME = "node-name"  # special access key for the dict
_GRAPHML_NODE_ATTRIBUTE_ID = "node-uuid"  # special access key for the dict
_GRAPHML_NODE_ATTRIBUTE_X_POSITION = "x-value"
_GRAPHML_NODE_ATTRIBUTE_Y_POSITION = "y-value"
_GRAPHML_NODE_ATTRIBUTE_POLYGON_POINTS = "node-polygon-points"

# GraphML edge constants
_GRAPHML_EDGE_TAG_EDGE = "edge"
_GRAPHML_EDGE_ATTRIBUTE_NAME = "edge-name"  # special access key for the dict
_GRAPHML_EDGE_ATTRIBUTE_ID = "edge-uuid"  # special access key for the dict
_GRAPHML_EDGE_ATTRIBUTE_SRC_NODE_ID = "v-src-uuid"
_GRAPHML_EDGE_ATTRIBUTE_DST_NODE_ID = "v-dest-uuid"
_GRAPHML_EDGE_ATTRIBUTE_DIRECTED = "directed"
_GRAPHML_EDGE_ATTRIBUTE_POLYGON_POINTS = "edge-polygon-points"

# Default velocity 1 m / s (meter per second)
_GRAPML_DEFAULT_EDGE_VELOCITY = 1.0

_EDGE_TYPE_DIRECTED = 1
_EDGE_TYPE_UNDIRECTED = 0

_EDGE_TYPE_OUTGOING = "ET_OUTGOING"
_EDGE_TYPE_INCOMING = "ET_INCOMING"

_GRAPHML_DEFAULT_NAMESPACE = ".//{http://graphml.graphdrawing.org/xmlns}"


class GrapMLParser(TopologyParser):
    def __init__(self, footprint_type=MARSFootprintType.MARS_FOOTPRINT_TYPE_SQUARE,
                 footprint_radius=DEFAULT_FOOTPRINT_RADIUS,
                 footprint_resolution=DEFAULT_FOOTPRINT_RESOLUTION):

        TopologyParser.__init__(self)

        self.__footprint_type = footprint_type
        self.__footprint_radius = footprint_radius
        self.__footprint_resolution = footprint_resolution
        self.__data_key_dict = dict()

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

        rospy.logdebug("[GraphMLParser][parse_file] Parsing file: " +
                       str(file_path))

        tree = ET.parse(file_path)

        if tree:
            root = tree.getroot()

            # test1 = root.findall('.//{http://graphml.graphdrawing.org/xmlns}node')

            # for bla in root.findall('{http://graphml.graphdrawing.org/xmlns}'+"graph"):
            #     for test in bla.findall('{http://graphml.graphdrawing.org/xmlns}node'):
            #         datas = test.findall('{http://graphml.graphdrawing.org/xmlns}data')
            #         for data in datas:
            #             print data.get('key')
            #             print data.text

            self.__data_key_dict = self.__parse_data_keys(root)
            self._mars_vertices = self.__parse_graphml_nodes(root)
            self._mars_edges = self.__parse_graphml_edges(root)

            self.print_parsed_topology_ros_debug()
        else:
            parsed_successfully = False
            rospy.logerr("[GraphMLParser][parse_file] Could not parse file!")

        return parsed_successfully

    def __parse_data_keys(self, root):
        data_key_dict = dict()

        for key in root.findall(_GRAPHML_DEFAULT_NAMESPACE + _GRAPHML_KEY_TAG_KEY):
            id = key.get(_GRAPHML_KEY_ATTRIBUTE_ID)
            attrName = key.get(_GRAPHML_KEY_ATTRIBUTE_NAME)
            forTag = key.get(_GRAPHML_KEY_FOR)

            # first check for clear attribute names for edge and vertex

            if attrName == _GRAPHML_NODE_ATTRIBUTE_X_POSITION:
                data_key_dict[_GRAPHML_NODE_ATTRIBUTE_X_POSITION] = id
            elif attrName == _GRAPHML_NODE_ATTRIBUTE_Y_POSITION:
                data_key_dict[_GRAPHML_NODE_ATTRIBUTE_Y_POSITION] = id
            elif attrName == _GRAPHML_EDGE_ATTRIBUTE_SRC_NODE_ID:
                data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_SRC_NODE_ID] = id
            elif attrName == _GRAPHML_EDGE_ATTRIBUTE_DST_NODE_ID:
                data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_DST_NODE_ID] = id
            elif attrName == _GRAPHML_EDGE_ATTRIBUTE_DIRECTED:
                data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_DIRECTED] = id
            else:
                if (forTag == _GRAPHML_KEY_TAG_FOR_NODE):
                    if attrName == _GRAPHML_ATTRIBUTE_NAME:
                        data_key_dict[_GRAPHML_NODE_ATTRIBUTE_NAME] = id
                    elif attrName == _GRAPHML_ATTRIBUTE_ID:
                        data_key_dict[_GRAPHML_NODE_ATTRIBUTE_ID] = id
                    elif attrName == _GRAPHML_ATTRIBUTE_POLYGON_POINTS:
                        data_key_dict[_GRAPHML_NODE_ATTRIBUTE_POLYGON_POINTS] = id
                    else:
                        rospy.logdebug("[GraphMLParser][__create_dict_from_data_keys]"
                                       " Unused id found: " + id + "(" + attrName + ")")
                elif (forTag == _GRAPHML_KEY_TAG_FOR_EDGE):
                    if attrName == _GRAPHML_ATTRIBUTE_NAME:
                        data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_NAME] = id
                    elif attrName == _GRAPHML_ATTRIBUTE_ID:
                        data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_ID] = id
                    elif attrName == _GRAPHML_ATTRIBUTE_POLYGON_POINTS:
                        data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_POLYGON_POINTS] = id
                    else:
                        rospy.logdebug("[GraphMLParser][__create_dict_from_data_keys]"
                                       " Unused id found: " + id + "(" + attrName + ")")
                else:
                    rospy.logdebug(
                        "[GraphMLParser][__create_dict_from_data_keys]"
                        " Unknown for tag found: " + forTag)

        return data_key_dict

    def __parse_graphml_nodes(self, root):
        """Parses all point objects in the GraphML file.

        Parses all point objects in the GraphML file and creates individual
        objects with needed attributes.

        Args:
            root: root element of the parsed GraphML xml tree.

        Returns:
            Returns a list of all parsed point objects.

        Raises:
        """
        mars_vertices = dict()
        footprint_x, footprint_y = None, None

        for graphml_node in root.findall(_GRAPHML_DEFAULT_NAMESPACE + _GRAPHML_NODE_TAG_NODE):
            for node_data in graphml_node.findall(
                    _GRAPHML_DEFAULT_NAMESPACE + _GRAPHML_DATA_TAG_DATA):

                data_key = node_data.get(_GRAPHML_DATA_ATTRIBUTE_KEY)

                if (data_key == self.__data_key_dict[_GRAPHML_NODE_ATTRIBUTE_NAME]):
                    node_name = node_data.text
                elif (data_key == self.__data_key_dict[_GRAPHML_NODE_ATTRIBUTE_ID]):
                    node_id = node_data.text
                elif (data_key == self.__data_key_dict[_GRAPHML_NODE_ATTRIBUTE_X_POSITION]):
                    node_x_pos = float(node_data.text)
                elif (data_key == self.__data_key_dict[_GRAPHML_NODE_ATTRIBUTE_Y_POSITION]):
                    node_y_pos = float(node_data.text)
                # Check now optional parameter
                elif ((_GRAPHML_NODE_ATTRIBUTE_POLYGON_POINTS in self.__data_key_dict.keys()) and (data_key == self.__data_key_dict[_GRAPHML_NODE_ATTRIBUTE_POLYGON_POINTS])):
                    if node_data.text:
                        footprint_x, footprint_y = self.__parse_footprint_string(node_data.text)
                    else:
                        rospy.logwarn("[GrapMLParser][__parse_graphml_nodes] Empty footprint for node was given (" + node_name + ")!")
                else:
                    rospy.logdebug(
                        "[GraphMLParser][__parse_graphml_nodes] Unused id found: " +
                        data_key + " (" + node_data.text + ")")

            # Create base node from msg
            mars_vertex = self._create_mars_vertex(
                node_name, node_x_pos, node_y_pos, self.__footprint_type, self.__footprint_radius,
                footprint_resolution=self.__footprint_resolution, uuid=node_id,
                footprint_x=footprint_x, footprint_y=footprint_y)

            mars_vertices[node_id] = mars_vertex

        return mars_vertices

    def __parse_graphml_edges(self, root):
        """Parses all path objects in the GraphML file.

        Parses all path objects in the GraphML file and creates individual
        objects with needed attributes. Also relates edges and vertices. Before
        you call this method, the GraphML points (__parse_graphml_edges) must
        be parsed and self._mars_vertices must be set!

        Args:
            root: root element of the parsed GraphML xml tree.

        Returns:
            Returns a list of all parsed path objects.

        Raises:
        """
        mars_edges = dict()
        footprint_x, footprint_y = None, None

        # check whether points of the topology were successfully parsed
        if len(self._mars_vertices):
            for graphml_edge in root.findall(_GRAPHML_DEFAULT_NAMESPACE + _GRAPHML_EDGE_TAG_EDGE):
                for edge_data in graphml_edge.findall(
                        _GRAPHML_DEFAULT_NAMESPACE + _GRAPHML_DATA_TAG_DATA):

                    data_key = edge_data.get(_GRAPHML_DATA_ATTRIBUTE_KEY)

                    if (data_key == self.__data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_NAME]):
                        edge_name = edge_data.text
                    elif (data_key == self.__data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_ID]):
                        edge_id = edge_data.text
                    elif (data_key == self.__data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_DIRECTED]):
                        edge_direction_type = int(edge_data.text)
                    elif (data_key == self.__data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_SRC_NODE_ID]):
                        edge_src_vertex_id = edge_data.text
                    elif (data_key == self.__data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_DST_NODE_ID]):
                        edge_dst_vertex_id = edge_data.text
                    # Check now optional parameter
                    elif ((_GRAPHML_EDGE_ATTRIBUTE_POLYGON_POINTS in self.__data_key_dict.keys()) and (data_key == self.__data_key_dict[_GRAPHML_EDGE_ATTRIBUTE_POLYGON_POINTS])):
                        if edge_data.text:
                            footprint_x, footprint_y = self.__parse_footprint_string(edge_data.text)
                        else:
                            rospy.logwarn("[GrapMLParser][__parse_graphml_edges] Empty footprint for edge was given (" + edge_name + ")!")
                    else:
                        rospy.logdebug(
                            "[GraphMLParser][__parse_graphml_edges] Unused id found: " +
                            data_key + " (" + edge_data.text + ")")

                mars_edge_length = math.sqrt(
                    math.pow((self._mars_vertices[edge_src_vertex_id].get_x_position(
                    ) - self._mars_vertices[edge_dst_vertex_id].get_x_position()), 2) +
                    math.pow((self._mars_vertices[edge_src_vertex_id].get_y_position(
                    ) - self._mars_vertices[edge_dst_vertex_id].get_y_position()), 2))

                try:
                    mars_edge = self.__check_for_edge_existence(
                        mars_edges, self._mars_vertices[edge_src_vertex_id].get_id(),
                        self._mars_vertices[edge_dst_vertex_id].get_id())
                    if mars_edge is None:
                        mars_edge = self._create_mars_edge(
                            edge_name, mars_edge_length, edge_id, footprint_x, footprint_y)
                        mars_edge.set_position(
                            ((self._mars_vertices[edge_src_vertex_id].get_x_position(
                            ) + self._mars_vertices[edge_dst_vertex_id].get_x_position()) / 2.0),
                            ((self._mars_vertices[edge_src_vertex_id].get_y_position(
                            ) + self._mars_vertices[edge_dst_vertex_id].get_y_position()) / 2.0))
                        mars_edges[edge_id] = mars_edge

                        mars_edge.add_path(
                            self._mars_vertices[edge_src_vertex_id].get_id(),
                            self._mars_vertices[edge_dst_vertex_id].get_id(),
                            _GRAPML_DEFAULT_EDGE_VELOCITY)

                        # Adding bidirectional edge
                        if (edge_direction_type == _EDGE_TYPE_UNDIRECTED):
                            mars_edge.add_path(
                                self._mars_vertices[edge_dst_vertex_id].get_id(),
                                self._mars_vertices[edge_src_vertex_id].get_id(),
                                _GRAPML_DEFAULT_EDGE_VELOCITY)

                        # Add edge to vertex 1 as in- and outgoing
                        self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                     edge_dst_vertex_id,
                                                     _EDGE_TYPE_INCOMING)
                        self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                     edge_src_vertex_id,
                                                     _EDGE_TYPE_OUTGOING)

                        # Add edge to vertex 2 as in- and outgoing
                        if (edge_direction_type == _EDGE_TYPE_UNDIRECTED):
                            self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                         edge_src_vertex_id,
                                                         _EDGE_TYPE_INCOMING)
                            self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                         edge_dst_vertex_id,
                                                         _EDGE_TYPE_OUTGOING)
                    else:
                        mars_edge.add_path(
                            self._mars_vertices[edge_src_vertex_id].get_id(),
                            self._mars_vertices[edge_dst_vertex_id].get_id(),
                            _GRAPML_DEFAULT_EDGE_VELOCITY)
                        self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                     edge_dst_vertex_id,
                                                     _EDGE_TYPE_INCOMING)
                        self.__add_edge_id_to_vertex(mars_edge.get_id(),
                                                     edge_src_vertex_id,
                                                     _EDGE_TYPE_OUTGOING)



                except TopologyException as err:
                    rospy.logwarn(str(err))
        else:
            rospy.logerr("[GraphMLParser][__create_mars_edges]"
                         " No vertices parsed!")

        return mars_edges

    def __add_edge_id_to_vertex(self, edge_id, vertex_name, edge_type):
        if vertex_name in self._mars_vertices:
            if edge_type == _EDGE_TYPE_INCOMING:
                self._mars_vertices[vertex_name].add_incoming_edge_id(edge_id)
            elif edge_type == _EDGE_TYPE_OUTGOING:
                self._mars_vertices[vertex_name].add_outgoing_edge_id(edge_id)
            else:
                rospy.logerr("[GraphMLParser][__add_edge_id_to_vertex]"
                             " Unsupported edge type given: " + edge_type)
        else:
            rospy.logerr(
                "[GraphMLParser][__add_edge_id_to_vertex]"
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
                        + " (" + str(path.get_origin_vertex_id().get_id())
                        + ") to "
                        + str(path.get_dst_vertex_id().get_description())
                        + " (" + str(path.get_dst_vertex_id().get_id())
                        + ") already exists in edge "
                        + str(edge.get_id().get_description())
                        + " (" + str(edge.get_id().get_id()) + ")!"))

        return existing_edge

    def __parse_footprint_string(self, poly_string):
        footprint_x = []
        footprint_y = []

        if poly_string:
            string_tuple = poly_string.split(") (")

            for tuple in string_tuple:
                tuple = tuple.replace('(', '')
                tuple = tuple.replace(')', '')
                x_y_part = tuple.split(', ')

                if len(x_y_part) != 2:
                    rospy.logwarn(
                        "[GrapMLParser][__parse_footprint_string] Error parsing footprint. Tuple of x y values not found in: " + string_tuple)
                else:
                    footprint_x.append(float(x_y_part[0]))
                    footprint_y.append(float(x_y_part[1]))
        else:
            rospy.logwarn("[GrapMLParser][__parse_footprint_string] Empty footprint was given!")

        return footprint_x, footprint_y

#!/usr/bin/env python
"""
    Fraunhofer IML
    Department Automation and Embedded Systems
    Tabsize          : 4
    Charset          : UTF-8
"""
__author__ = "Dennis Luensch"
__maintainer__ = "Dennis Luensch"
__email__ = "dennis.luensch@iml.fraunhofer"

from MARSVertex import MARSVertex
from MARSEdge import MARSEdge
import MARSTopologieNode

import rospy
import os
import re


_EDGE_ID_PREFIX = "edge_"
_VERTEX_ID_PREFIX = "vertex_"


class MARSNodeLauncher(MARSTopologieNode.MARSTopologieNode):

    def __init__(self):
        MARSTopologieNode.MARSTopologieNode.__init__(self)

    def launch_mars_vertices(self, mars_vertices, node_pkg,
                             node_type, node_ns_prefix,
                             default_entity_max_linear_velocity,
                             default_entity_max_angular_velocity,
                             default_entity_max_linear_acceleration,
                             default_entity_max_angular_acceleration):
        """ Starts ros nodes for given mars_vertices.

        Starts ros nodes for given mars_vertices.

        Args:
            mars_vertices: mars vertices which has to be started.
            node_pkg: name of the ros package.     
            node_type: file inside the package which has to be started.
            node_ns_prefix: generic prefix for the ros nodes.

        Returns:

        Raises:
            Raises a 'NodeLaunchException' if a node can't be started.
        """

        for mars_vertex in mars_vertices.values():

            ns = str(node_ns_prefix)
            node_name = re.sub(
                "[-]", "", (_VERTEX_ID_PREFIX + str(mars_vertex.get_id().get_id())))

            max_linear_velocity = mars_vertex.get_max_linear_velocity(
            ) if mars_vertex.get_max_linear_velocity() is not None else default_entity_max_linear_velocity

            max_angular_velocity = mars_vertex.get_max_angular_velocity(
            ) if mars_vertex.get_max_angular_velocity() is not None else default_entity_max_angular_velocity

            max_linear_acceleration = mars_vertex.get_max_linear_acceleration(
            ) if mars_vertex.get_max_linear_acceleration() is not None else default_entity_max_linear_acceleration

            max_angular_acceleration = mars_vertex.get_max_angular_acceleration(
            ) if mars_vertex.get_max_angular_acceleration() is not None else default_entity_max_angular_acceleration

            # set general parameters
            self._set_general_params_on_parameter_server(mars_vertex,
                                                         node_name, ns)

            # create incoming and outgoing edge id arrays
            ingoing_edge_ids = self._create_string_ids_from_id_array(
                mars_vertex.get_incoming_edge_ids())
            outgoing_edge_ids = self._create_string_ids_from_id_array(
                mars_vertex.get_outgoing_edge_ids())

            # set vertex specific parameters
            self._set_param_on_parameter_server(
                ingoing_edge_ids,
                MARSTopologieNode._PARAM_NAME_INGOING_EDGE_IDS, node_name, ns)

            self._set_param_on_parameter_server(
                outgoing_edge_ids,
                MARSTopologieNode._PARAM_NAME_OUTGOING_EDGE_IDS, node_name, ns)

            self._set_param_on_parameter_server(
                mars_vertex.get_footprintX(),
                MARSTopologieNode._PARAM_NAME_FOOTPRINT_X, node_name, ns)

            self._set_param_on_parameter_server(
                mars_vertex.get_footprintY(),
                MARSTopologieNode._PARAM_NAME_FOOTPRINT_Y, node_name, ns)

            self._set_param_on_parameter_server(
                max_linear_velocity, MARSTopologieNode._PARAM_NAME_MAXIMUM_LINEAR_VELOCITY,
                node_name, ns)
            self._set_param_on_parameter_server(
                max_angular_velocity, MARSTopologieNode._PARAM_NAME_MAXIMUM_ANGULAR_VELOCITY,
                node_name, ns)
            self._set_param_on_parameter_server(
                max_linear_acceleration, MARSTopologieNode._PARAM_NAME_MAXIMUM_LINEAR_ACCELERATION,
                node_name, ns)
            self._set_param_on_parameter_server(
                max_angular_acceleration, MARSTopologieNode._PARAM_NAME_MAXIMUM_ANGULAR_ACCELERATION, node_name, ns)

            self._run_node(node_pkg, node_type, node_name, ns)

    def launch_mars_edges(self, mars_edges, node_pkg,
                          node_type, node_ns_prefix,
                          default_entity_max_linear_velocity,
                          default_entity_max_angular_velocity,
                          default_entity_max_linear_acceleration,
                          default_entity_max_angular_acceleration):
        """ Starts ros nodes for given mars_edges.

        Starts ros nodes for given mars_edges.

        Args:
            mars_edges: mars edges which has to be started.
            node_pkg: name of the ros package.     
            node_type: file inside the package which has to be started.
            node_ns_prefix: generic prefix for the ros nodes.

        Returns:

        Raises:
            Raises a 'NodeLaunchException' if a node can't be started.
        """
        origin_vertex_ids = []
        dst_vertex_ids = []
        edge_velocities = []

        for mars_edge in mars_edges.values():

            ns = str(node_ns_prefix)
            node_name = re.sub(
                "[-]", "", (_EDGE_ID_PREFIX + str(mars_edge.get_id().get_id())))

            # set general parameters
            self._set_general_params_on_parameter_server(mars_edge,
                                                         node_name, ns)

            origin_vertex_ids, dst_vertex_ids, edge_velocities = \
                self._create_arrays_from_mars_paths(mars_edge.get_paths())

            # create incoming and outgoing edge id arrays
            str_origin_vertex_ids = self._create_string_ids_from_id_array(
                origin_vertex_ids)
            str_dst_vertex_ids = self._create_string_ids_from_id_array(
                dst_vertex_ids)

            edge_length = mars_edge.get_length()

            max_linear_velocity = mars_edge.get_max_linear_velocity(
            ) if mars_edge.get_max_linear_velocity() is not None else default_entity_max_linear_velocity

            max_angular_velocity = mars_edge.get_max_angular_velocity(
            ) if mars_edge.get_max_angular_velocity() is not None else default_entity_max_angular_velocity

            max_linear_acceleration = mars_edge.get_max_linear_acceleration(
            ) if mars_edge.get_max_linear_acceleration() is not None else default_entity_max_linear_acceleration

            max_angular_acceleration = mars_edge.get_max_angular_acceleration(
            ) if mars_edge.get_max_angular_acceleration() is not None else default_entity_max_angular_acceleration

            footprintX = mars_edge.get_footprintX() if mars_edge.get_footprintX() else []
            footprintY = mars_edge.get_footprintY() if mars_edge.get_footprintY() else []

            # Check if edge is uni or bidirectional
            if (len(str_origin_vertex_ids) == 2):
                edge_type = MARSTopologieNode._EDGE_DIRECTION_BIDIRECTIONAL
            else:
                edge_type = MARSTopologieNode._EDGE_DIRECTION_UNIDIRECTIONAL

            # set edge specific parameters
            self._set_param_on_parameter_server(
                str_origin_vertex_ids[0],
                MARSTopologieNode._PARAM_NAME_ORIGIN_VERTEX_ID, node_name, ns)
            self._set_param_on_parameter_server(
                str_dst_vertex_ids[0],
                MARSTopologieNode._PARAM_NAME_DESTINATION_VERTEX_ID, node_name, ns)
            self._set_param_on_parameter_server(
                edge_type,
                MARSTopologieNode._PARAM_NAME_DIRECTION, node_name, ns)
            self._set_param_on_parameter_server(
                edge_velocities,
                MARSTopologieNode._PARAM_NAME_VELOCITIES, node_name, ns)
            self._set_param_on_parameter_server(
                edge_length, MARSTopologieNode._PARAM_NAME_LENGTH, node_name, ns)

            self._set_param_on_parameter_server(
                max_linear_velocity, MARSTopologieNode._PARAM_NAME_MAXIMUM_LINEAR_VELOCITY,
                node_name, ns)
            self._set_param_on_parameter_server(
                max_angular_velocity, MARSTopologieNode._PARAM_NAME_MAXIMUM_ANGULAR_VELOCITY,
                node_name, ns)
            self._set_param_on_parameter_server(
                max_linear_acceleration, MARSTopologieNode._PARAM_NAME_MAXIMUM_LINEAR_ACCELERATION,
                node_name, ns)
            self._set_param_on_parameter_server(
                max_angular_acceleration, MARSTopologieNode._PARAM_NAME_MAXIMUM_ANGULAR_ACCELERATION, node_name, ns)
            self._set_param_on_parameter_server(
                footprintX, MARSTopologieNode._PARAM_NAME_FOOTPRINT_X, node_name, ns)
            self._set_param_on_parameter_server(
                footprintY, MARSTopologieNode._PARAM_NAME_FOOTPRINT_Y, node_name, ns)

            self._run_node(node_pkg, node_type, node_name, ns)

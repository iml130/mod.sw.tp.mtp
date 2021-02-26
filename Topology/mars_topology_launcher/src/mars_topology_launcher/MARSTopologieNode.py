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

from MARSEntity import MARSEntity
from NodeLaunchException import NodeLaunchException

import rospy
import roslaunch

# General mars node parameter
_PARAM_NAME_ID = "id"
_PARAM_NAME_DESCRIPTION = "description"
_PARAM_NAME_X_POS = "x_pos"
_PARAM_NAME_Y_POS = "y_pos"
_PARAM_NAME_IS_LOCKED = "is_locked"
_PARAM_NAME_TYPE = "type"
_PARAM_NAME_FRAME_ID = "frame_id"
_PARAM_NAME_NODE_RATE = "node_rate"
_PARAM_NAME_DRAWING_RATE = "drawing_rate"
_PARAM_NAME_FORBIDDEN_HAZARD_TYPES = "forbidden_hazard_types"
_PARAM_NAME_FORBIDDEN_VEHICLE_TYPES = "forbidden_vehicle_types"
_PARAM_NAME_MAXIMUM_HEIGHT = "maximum_height"
_PARAM_NAME_MAXIMUM_TOTAL_WEIGHT = "maximum_total_weight"

_TOPOLOGY_NODE_RATE = 1000
_TOPOLOGY_DRAWING_RATE = 0.1
_TOPOLOGY_FRAME_ID = "map"

# Vertex specific parameters
_PARAM_NAME_INGOING_EDGE_IDS = "ingoing_edge_ids"
_PARAM_NAME_OUTGOING_EDGE_IDS = "outgoing_edge_ids"

# Edge specific parameters
_PARAM_NAME_LENGTH = "length"
_PARAM_NAME_ORIGIN_VERTEX_ID = "origin_id"
_PARAM_NAME_DESTINATION_VERTEX_ID = "destination_id"
_PARAM_NAME_DIRECTION = "direction"
_PARAM_NAME_VELOCITIES = "velocities"
_PARAM_NAME_FOOTPRINT_X = "footprint_x"
_PARAM_NAME_FOOTPRINT_Y = "footprint_y"
_PARAM_NAME_MAXIMUM_LINEAR_VELOCITY = "maximum_linear_velocity"
_PARAM_NAME_MAXIMUM_ANGULAR_VELOCITY = "maximum_angular_velocity"
_PARAM_NAME_MAXIMUM_LINEAR_ACCELERATION = "maximum_linear_acceleration"
_PARAM_NAME_MAXIMUM_ANGULAR_ACCELERATION = "maximum_angular_acceleration"

_EDGE_DIRECTION_UNIDIRECTIONAL = 0
_EDGE_DIRECTION_BIDIRECTIONAL = 2


class MARSTopologieNode():

    def __init__(self):
        self.__launch = roslaunch.scriptapi.ROSLaunch()
        self.__launch.start()

    def _set_general_params_on_parameter_server(self, mars_entity,
                                                node_name, ns):

        self._set_param_on_parameter_server(str(mars_entity.get_id().get_id()),
                                            _PARAM_NAME_ID, node_name, ns)
        self._set_param_on_parameter_server(mars_entity.get_id().get_description(),
                                            _PARAM_NAME_DESCRIPTION, node_name, ns)
        self._set_param_on_parameter_server(float(mars_entity.get_x_position()),
                                            _PARAM_NAME_X_POS, node_name, ns)
        self._set_param_on_parameter_server(mars_entity.get_y_position(),
                                            _PARAM_NAME_Y_POS, node_name, ns)
        self._set_param_on_parameter_server(mars_entity.get_lock(),
                                            _PARAM_NAME_IS_LOCKED, node_name, ns)
        self._set_param_on_parameter_server(mars_entity.get_type().get_value(),
                                            _PARAM_NAME_TYPE, node_name, ns)
        self._set_param_on_parameter_server(_TOPOLOGY_NODE_RATE,
                                            _PARAM_NAME_NODE_RATE, node_name, ns)

    def _set_param_on_parameter_server(self, value, param_name, node_name, ns=""):
        # Add beginning slash to ns
        if len(ns) > 0 and not ns[:1] == '/':
            ns = '/' + ns
        rospy.logdebug("set_param: " + param_name)
        rospy.set_param(ns + '/' + node_name + '/' + param_name, value)

    def _create_string_ids_from_id_array(self, id_array):
        string_id_array = []

        for id in id_array:
            string_id_array.append(str(id.get_id()))

        return string_id_array

    def _create_arrays_from_mars_paths(self, mars_paths):
        origin_vertex_ids = []
        dst_vertex_ids = []
        edge_velocities = []

        for mars_path in mars_paths:
            origin_vertex_ids.append(mars_path.get_origin_vertex_id())
            dst_vertex_ids.append(mars_path.get_dst_vertex_id())
            edge_velocities.append(mars_path.get_max_velocity())

        return origin_vertex_ids, dst_vertex_ids, edge_velocities

    def _run_node(self, node_pkg, node_type, node_name, ns):
        """ Starts a ros node.

        Starts a ros node inside a given namespace etc. 

        Args:
            node_pkg: name of the ros package.     
            node_type: file inside the package which has to be started.
            node_name: name of the node.
            ns: namespace for the node which has to be started.

        Returns:

        Raises:
            Raises a 'NodeLaunchException' if the node can't be started.
        """
        node = roslaunch.core.Node(
            node_pkg, node_type, name=node_name, namespace=ns)

        process = self.__launch.launch(node)
        rospy.logdebug("[class MARSTopologieNode][_run_node] Launched node: " + ns )

        if not process.is_alive():
            raise NodeLaunchException(
                "[class MARSTopologieNode][_run_node] Can't run ros node: " + os.linesep
                + "ns: " + str(ns) + os.linesep
                         + "node_pkg: " + str(node_pkg) + os.linesep
                         + "node_name: " + str(node_name) + os.linesep
                         + "node_type: " + str(node_type))

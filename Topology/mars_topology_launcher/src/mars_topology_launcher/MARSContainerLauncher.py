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
from mars_common.Id import Id, IdType
from NodeLaunchException import NodeLaunchException

import MARSTopologieNode
import rospy
import roslaunch
import re
from enum import Enum

try:
    set
except NameError:
    from sets import Set as set

_CONTAINER_SPLIT_MODE_RANDOM_ENTITY_COUNT_INFINITE = -1

_CONTAINER_ID_PREFIX = "container_"

_PARAM_NAME_CONTAINER = "container"
_PARAM_NAME_CONTAINER_ID = "container_id"
_PARAM_NAME_ENTITY_TYPE = "entity_type"
_PARAM_NAME_ALLOCATE_TOPIC_NAME = "allocate"

# Container specific params
_PARAM_NAME_ACTIONLIB_CLIENT_SUB_QUEUE_SIZE = "actionlib_client_sub_queue_size"
_PARAM_NAME_ACTIONLIB_CLIENT_PUB_QUEUE_SIZE = "actionlib_client_pub_queue_size"
_PARAM_NAME_ACTIONLIB_SERVER_SUB_QUEUE_SIZE = "actionlib_server_sub_queue_size"
_PARAM_NAME_ACTIONLIB_SERVER_PUB_QUEUE_SIZE = "actionlib_server_pub_queue_size"
_ACTIONLIB_CLIENT_SUB_QUEUE_SIZE = 100
_ACTIONLIB_CLIENT_PUB_QUEUE_SIZE = 100
_ACTIONLIB_SERVER_SUB_QUEUE_SIZE = 100
_ACTIONLIB_SERVER_PUB_QUEUE_SIZE = 100

# Vertex specific container params
_PARAM_NAME_VERTEX_INGOING_CONTAINER_IDS = "ingoing_container_ids"
_PARAM_NAME_VERTEX_OUTGOING_CONTAINER_IDS = "outgoing_container_ids"

# Edge specific container params
_PARAM_NAME_EDGE_ORIGIN_CONTAINER_ID = "origin_container_id"
_PARAM_NAME_EDGE_DESTINATION_CONTAINER_ID = "destination_container_id"

_MARS_CONTAINER_ENTITY_TYPE_VERTEX = "vertex"
_MARS_CONTAINER_ENTITY_TYPE_EDGE = "edge"


class MARSContainerSplitMode(Enum):

    MARS_CONTAINER_SPLIT_MODE_UNKNOWN = 0
    MARS_CONTAINER_SPLIT_MODE_RANDOM = 1


class MARSContainerLauncher(MARSTopologieNode.MARSTopologieNode):

    def __init__(self):
        MARSTopologieNode.MARSTopologieNode.__init__(self)
        self.__launch = roslaunch.scriptapi.ROSLaunch()
        self.__launch.start()

        self.__split_mode = None
        self.__split_mode_random_entity_count = None
        self.__container_spreading = dict()

    def set_split_mode_random(self, entity_split_count):
        self.__split_mode = MARSContainerSplitMode.MARS_CONTAINER_SPLIT_MODE_RANDOM

        if (entity_split_count > 0) or (entity_split_count == _CONTAINER_SPLIT_MODE_RANDOM_ENTITY_COUNT_INFINITE):
            self.__split_mode_random_entity_count = entity_split_count
        else:
            self.__split_mode_random_entity_count = _CONTAINER_SPLIT_MODE_RANDOM_ENTITY_COUNT_INFINITE
            rospy.logwarn(
                "[MARSContainerLauncher][set_split_mode_random] Unsupported value for entity_split_count set ("
                + entity_split_count +
                "). Supported values are -1 for infinite entity or a value > 0.")

    def launch_container(self, mars_vertices, mars_edges,
                         node_pkg, node_type, node_ns_prefix,
                         default_max_linear_velocity,
                         default_max_angular_velocity,
                         default_max_linear_acceleration,
                         default_max_angular_acceleration):

        if self.__split_mode == MARSContainerSplitMode.MARS_CONTAINER_SPLIT_MODE_RANDOM:
            self.__launch_container_split_mode_random(mars_vertices, mars_edges,
                                                      node_pkg, node_type, node_ns_prefix,
                                                      default_max_linear_velocity,
                                                      default_max_angular_velocity,
                                                      default_max_linear_acceleration,
                                                      default_max_angular_acceleration)
        else:
            rospy.logerr(
                "[MARSContainerLauncher][launch_container] Unsupported MARSContainerSplitMode set: "
                + self.__split_mode.name)

    def __launch_container_split_mode_random(self, mars_vertices, mars_edges,
                                             node_pkg, node_type, node_ns_prefix,
                                             default_entity_max_linear_velocity,
                                             default_entity_max_angular_velocity,
                                             default_entity_max_linear_acceleration,
                                             default_entity_max_angular_acceleration):
        merged_topology_entities = list()
        topology_dict = dict()
        container_id = None
        ns = str(node_ns_prefix)
        loop_run = 0
        container_name = "container_" + str(loop_run)
        container_id = Id(
            id=container_name, description=container_name,
            id_type=IdType.ID_TYPE_STRING_NAME)
        node_name = re.sub(
            "[-]", "", (_CONTAINER_ID_PREFIX + str(container_id.get_id())))
        id_set = set()

        topology_dict = dict(mars_vertices)
        topology_dict.update(mars_edges)

        # First create the set(s) of container (how to split)
        if self.__split_mode_random_entity_count == _CONTAINER_SPLIT_MODE_RANDOM_ENTITY_COUNT_INFINITE:

            for topology_entity in topology_dict.values():
                id_set.add(str(topology_entity.get_id().get_id()))
            self.__container_spreading[str(container_id.get_id())] = id_set
        else:
            i = 1

            for topology_entity in topology_dict.values():
                id_set.add(str(topology_entity.get_id().get_id()))

                if i == self.__split_mode_random_entity_count:
                    self.__container_spreading[str(container_id.get_id())] = id_set

                    # prepare next loop run
                    id_set = set()
                    i = 1
                    loop_run = loop_run + 1
                    container_name = "container_" + str(loop_run)
                    container_id = Id(
                        id=container_name, description=container_name,
                        id_type=IdType.ID_TYPE_STRING_NAME)
                else:
                    i = i + 1

            if len(id_set) > 0:
                self.__container_spreading[str(container_id.get_id())] = id_set

        merged_topology_entities = self.__create_list_of_topology_entities(
            mars_vertices, mars_edges, default_entity_max_linear_velocity,
            default_entity_max_angular_velocity, default_entity_max_linear_acceleration,
            default_entity_max_angular_acceleration)

        self._set_param_on_parameter_server(
            str(container_id.get_id()),
            _PARAM_NAME_CONTAINER_ID, node_name, ns)
        self._set_param_on_parameter_server(
            merged_topology_entities, _PARAM_NAME_CONTAINER, node_name, ns)

        self._set_param_on_parameter_server(
             _ACTIONLIB_CLIENT_SUB_QUEUE_SIZE,
             _PARAM_NAME_ALLOCATE_TOPIC_NAME + "/" + _PARAM_NAME_ACTIONLIB_CLIENT_SUB_QUEUE_SIZE, node_name, ns)
        self._set_param_on_parameter_server(
             _ACTIONLIB_CLIENT_PUB_QUEUE_SIZE,
             _PARAM_NAME_ALLOCATE_TOPIC_NAME + "/" + _PARAM_NAME_ACTIONLIB_CLIENT_PUB_QUEUE_SIZE, node_name, ns)
        self._set_param_on_parameter_server(
             _ACTIONLIB_SERVER_SUB_QUEUE_SIZE,
             _PARAM_NAME_ALLOCATE_TOPIC_NAME + "/" + _PARAM_NAME_ACTIONLIB_SERVER_SUB_QUEUE_SIZE, node_name, ns)
        self._set_param_on_parameter_server(
             _ACTIONLIB_SERVER_PUB_QUEUE_SIZE,
             _PARAM_NAME_ALLOCATE_TOPIC_NAME + "/" + _PARAM_NAME_ACTIONLIB_SERVER_PUB_QUEUE_SIZE, node_name, ns)

        self._run_node(node_pkg, node_type, node_name, ns)

        # for i in range(self.__split_mode_random_entity_count):

    def __create_list_of_topology_entities(self, mars_vertices, mars_edges,
                                           default_entity_max_linear_velocity,
                                           default_entity_max_angular_velocity,
                                           default_entity_max_linear_acceleration,
                                           default_entity_max_angular_acceleration):
        mars_entity_list = list()

        for vertex in mars_vertices.values():
            mars_entity_list.append(self.__create_dict_from_vertex(
                vertex, default_entity_max_linear_velocity,
                default_entity_max_angular_velocity,
                default_entity_max_linear_acceleration,
                default_entity_max_angular_acceleration))

        for edge in mars_edges.values():
            mars_entity_list.append(self.__create_dict_from_edge(
                edge, default_entity_max_linear_velocity,
                default_entity_max_angular_velocity,
                default_entity_max_linear_acceleration,
                default_entity_max_angular_acceleration))

        return mars_entity_list

    def __create_dict_from_vertex(self, mars_vertex,
                                  default_entity_max_linear_velocity,
                                  default_entity_max_angular_velocity,
                                  default_entity_max_linear_acceleration,
                                  default_entity_max_angular_acceleration):
        vertex_dict = self.__create_dict_from_mars_topology_entity(
            mars_vertex, default_entity_max_linear_velocity, default_entity_max_angular_velocity,
            default_entity_max_linear_acceleration, default_entity_max_angular_acceleration)

        ingoing_edge_id_container_list = self._create_string_ids_from_id_array(
            mars_vertex.get_incoming_edge_ids())
        outgoing_edge_id_container_list = self._create_string_ids_from_id_array(
            mars_vertex.get_outgoing_edge_ids())

        # set vertex specific parameters
        vertex_dict[MARSTopologieNode._PARAM_NAME_INGOING_EDGE_IDS] = ingoing_edge_id_container_list
        vertex_dict[_PARAM_NAME_VERTEX_INGOING_CONTAINER_IDS] = self.__create_container_link_list_vertex(
            ingoing_edge_id_container_list)
        vertex_dict[MARSTopologieNode._PARAM_NAME_OUTGOING_EDGE_IDS] = outgoing_edge_id_container_list
        vertex_dict[_PARAM_NAME_VERTEX_OUTGOING_CONTAINER_IDS] = self.__create_container_link_list_vertex(
            outgoing_edge_id_container_list)
        vertex_dict[MARSTopologieNode._PARAM_NAME_FOOTPRINT_X] = mars_vertex.get_footprintX()
        vertex_dict[MARSTopologieNode._PARAM_NAME_FOOTPRINT_Y] = mars_vertex.get_footprintY()

        return vertex_dict

    def __create_dict_from_edge(self, mars_edge,
                                default_entity_max_linear_velocity,
                                default_entity_max_angular_velocity,
                                default_entity_max_linear_acceleration,
                                default_entity_max_angular_acceleration):
        edge_dict = self.__create_dict_from_mars_topology_entity(
            mars_edge, default_entity_max_linear_velocity, default_entity_max_angular_velocity,
            default_entity_max_linear_acceleration, default_entity_max_angular_acceleration)
        origin_vertex_ids = []
        dst_vertex_ids = []
        edge_velocities = []
        footprintX = mars_edge.get_footprintX() if mars_edge.get_footprintX() else []
        footprintY = mars_edge.get_footprintY() if mars_edge.get_footprintY() else []

        origin_vertex_ids, dst_vertex_ids, edge_velocities = \
            self._create_arrays_from_mars_paths(mars_edge.get_paths())

        # create incoming and outgoing edge id arrays
        str_origin_vertex_ids = self._create_string_ids_from_id_array(
            origin_vertex_ids)
        str_dst_vertex_ids = self._create_string_ids_from_id_array(
            dst_vertex_ids)

        # Check if edge is uni or bidirectional
        if (len(str_origin_vertex_ids) == 2):
            edge_type = MARSTopologieNode._EDGE_DIRECTION_BIDIRECTIONAL
        else:
            edge_type = MARSTopologieNode._EDGE_DIRECTION_UNIDIRECTIONAL

        # set vertex specific parameters
        edge_dict[MARSTopologieNode._PARAM_NAME_ORIGIN_VERTEX_ID] = str_origin_vertex_ids[0]
        edge_dict[_PARAM_NAME_EDGE_ORIGIN_CONTAINER_ID] = self.__create_container_link_list_vertex(
            str_origin_vertex_ids[0:1])[0]
        edge_dict[MARSTopologieNode._PARAM_NAME_DESTINATION_VERTEX_ID] = str_dst_vertex_ids[0]
        edge_dict[_PARAM_NAME_EDGE_DESTINATION_CONTAINER_ID] = self.__create_container_link_list_vertex(
            str_dst_vertex_ids[0:1])[0]
        edge_dict[MARSTopologieNode._PARAM_NAME_DIRECTION] = edge_type
        edge_dict[MARSTopologieNode._PARAM_NAME_VELOCITIES] = edge_velocities
        edge_dict[MARSTopologieNode._PARAM_NAME_FOOTPRINT_X] = footprintX
        edge_dict[MARSTopologieNode._PARAM_NAME_FOOTPRINT_Y] = footprintY
        edge_dict[MARSTopologieNode._PARAM_NAME_LENGTH] = mars_edge.get_length()

        return edge_dict

    def __create_dict_from_mars_topology_entity(self, mars_topology_entity,
                                                default_entity_max_linear_velocity,
                                                default_entity_max_angular_velocity,
                                                default_entity_max_linear_acceleration,
                                                default_entity_max_angular_acceleration):
        mars_topology_entity_dict = dict()
        entity_type = ""

        if (isinstance(mars_topology_entity, MARSVertex)):
            entity_type = _MARS_CONTAINER_ENTITY_TYPE_VERTEX
        elif (isinstance(mars_topology_entity, MARSEdge)):
            entity_type = _MARS_CONTAINER_ENTITY_TYPE_EDGE
        else:
            raise NodeLaunchException(
                "[MARSContainerLauncher][__create_dict_from_mars_topology_entity] Unknow topology entity type! Type can only be 'MARSVertex' or 'MARSEdge'!")

        max_linear_velocity = mars_topology_entity.get_max_linear_velocity(
        ) if mars_topology_entity.get_max_linear_velocity() is not None else default_entity_max_linear_velocity

        max_angular_velocity = mars_topology_entity.get_max_angular_velocity(
        ) if mars_topology_entity.get_max_angular_velocity() is not None else default_entity_max_angular_velocity

        max_linear_acceleration = mars_topology_entity.get_max_linear_acceleration(
        ) if mars_topology_entity.get_max_linear_acceleration() is not None else default_entity_max_linear_acceleration

        max_angular_acceleration = mars_topology_entity.get_max_angular_acceleration(
        ) if mars_topology_entity.get_max_angular_acceleration() is not None else default_entity_max_angular_acceleration

        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_ID] = str(
            mars_topology_entity.get_id().get_id())
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_DESCRIPTION] = str(
            mars_topology_entity.get_id().get_description())
        mars_topology_entity_dict[_PARAM_NAME_ENTITY_TYPE] = entity_type
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_FRAME_ID] = MARSTopologieNode._TOPOLOGY_FRAME_ID
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_X_POS] = float(
            mars_topology_entity.get_x_position())
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_Y_POS] = float(
            mars_topology_entity.get_y_position())
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_IS_LOCKED] = mars_topology_entity.get_lock(
        )
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_TYPE] = mars_topology_entity.get_type(
        ).get_value()
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_FORBIDDEN_HAZARD_TYPES] = mars_topology_entity.get_forbidden_hazard_types()
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_FORBIDDEN_VEHICLE_TYPES] = mars_topology_entity.get_forbidden_vehicle_types()
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_MAXIMUM_HEIGHT] = mars_topology_entity.get_maximum_height(
        )
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_MAXIMUM_TOTAL_WEIGHT] = mars_topology_entity.get_maximum_total_weight()
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_MAXIMUM_LINEAR_VELOCITY] = max_linear_velocity
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_MAXIMUM_ANGULAR_VELOCITY] = max_angular_velocity
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_MAXIMUM_LINEAR_ACCELERATION] = max_linear_acceleration
        mars_topology_entity_dict[MARSTopologieNode._PARAM_NAME_MAXIMUM_ANGULAR_ACCELERATION] = max_angular_acceleration

        return mars_topology_entity_dict

    def __create_container_link_list_vertex(self, edge_id_list):
        link_list = list()

        for edge_id in edge_id_list:
            link_list.append(self.__get_container_id_for_entity(edge_id))

        return link_list

    def __get_container_id_for_entity(self, entity_id):
        found_entity = False
        container_id_str = ""

        for container_id, entity_id_set in self.__container_spreading.items():
            if (entity_id in entity_id_set):
                container_id_str = str(container_id)
                found_entity = True

        if not found_entity:
            raise NodeLaunchException(
                "[MARSContainerLauncher][__create_container_link_list] Edge Id can not be matched to all existing Id's.")

        return container_id_str

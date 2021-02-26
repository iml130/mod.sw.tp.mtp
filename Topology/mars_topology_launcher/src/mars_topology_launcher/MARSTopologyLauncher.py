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


import rospy

from OpenTCSParser import OpenTCSParser
from GraphMLParser import GrapMLParser
from MARSNodeLauncher import MARSNodeLauncher
from ReadParamException import ReadParamException
from NodeLaunchException import NodeLaunchException
from MARSVertex import MARSFootprintType
import MARSContainerLauncher

_OPIL_PARSER_AVAILABLE = True

try:
    from OpilSPParser import OpilSPParser
except:
    _OPIL_PARSER_AVAILABLE = False
    rospy.logwarn("[MARSTopologyLauncher][]"
                  " Opil Parser not available. Needed MSGs can't be found!")


_NODE_NAME = "mars_topology_launcher"

_NODE_RATE = 1

_LOG_LEVEL_DEBUG = "debug"
_LOG_LEVEL_INFO = "info"
_LOG_LEVEL_WARN = "warn"
_LOG_LEVEL_ERROR = "error"
_LOG_LEVEL_FATAL = "fatal"

_TOPOLOGY_LAUNCH_MODE_AGENT = "agent"
_TOPOLOGY_LAUNCH_MODE_CONTAINER = "container"

_TOPOLOGY_FILE_TYPE_OPENTCS = "opentcs"
_TOPOLOGY_FILE_TYPE_OPIL_SP = "opil_sp"
_TOPOLOGY_FILE_TYPE_GRAPHML = "graphml"

_TOPOLOGY_TOPIC_NAME = "topology"

# Params with default value
_STD_PARAM_NAME_LOG_LEVEL = "log_level"
_STD_PARAM_LOG_LEVEL = _LOG_LEVEL_INFO

_STD_PARAM_NAME_TOPOLOGY_FILE_TYPE = "topo_file_type"
_STD_PARAM_TOPOLOGY_FILE_TYPE = _TOPOLOGY_FILE_TYPE_OPENTCS

_STD_PARAM_NAME_TOPO_TOPIC_NAME = "topo_topic_name"
_STD_PARAM_TOPO_TOPIC_NAME = _TOPOLOGY_TOPIC_NAME

_STD_PARAM_NAME_TOPOLOGY_LAUNCH_MODE = "topologie_launch_mode"
_STD_PARAM_TOPOLOGY_LAUNCH_MODE = _TOPOLOGY_LAUNCH_MODE_CONTAINER

# Footprint BB type
_STD_PARAM_NAME_VERTEX_FOOTPRINT_BB_TYPE = "mars_vertex_footprint_bb_type"
_STD_PARAM_VERTEX_FOOTPRINT_BB_TYPE = MARSFootprintType.MARS_FOOTPRINT_TYPE_CIRCLE

# Radius in meter
_STD_PARAM_NAME_VERTEX_FOOTPRINT_RADIUS = "mars_vertex_footprint_radius"

# Resolution in number of points
_STD_PARAM_NAME_VERTEX_FOOTPRINT_RESOLUTION = "mars_vertex_footprint_resolution"

# Params without default value
_STD_PARAM_NAME_TOPOLOGY_FILE_PATH = "topo_file_path"

# Params entity (generic)
_STD_PARAM_NAME_ENTITY_MAX_LINEAR_VELOCITY = "mars_entity_maximum_linear_velocity"
_STD_PARAM_NAME_ENTITY_MAX_ANGULAR_VELOCITY = "mars_entity_maximum_angular_velocity"
_STD_PARAM_NAME_ENTITY_MAX_LINEAR_ACCELERATION = "mars_entity_maximum_linear_acceleration"
_STD_PARAM_NAME_ENTITY_MAX_ANGULAR_ACCELERATION = "mars_entity_maximum_angular_acceleration"

# Params vertex
_STD_PARAM_NAME_VERTEX_NODE_PKG = "mars_vertex_node_pkg"
_STD_PARAM_NAME_VERTEX_NODE_TYPE = "mars_vertex_node_type"
_STD_PARAM_NAME_VERTEX_NODE_NS_PREFIX = "mars_vertex_node_ns_prefix"

# Params edge
_STD_PARAM_NAME_EDGE_NODE_PKG = "mars_edge_node_pkg"
_STD_PARAM_NAME_EDGE_NODE_TYPE = "mars_edge_node_type"
_STD_PARAM_NAME_EDGE_NODE_NS_PREFIX = "mars_edge_node_ns_prefix"

# Params container
_STD_PARAM_NAME_CONTAINER_NODE_PKG = "mars_container_node_pkg"
_STD_PARAM_NAME_CONTAINER_NODE_TYPE = "mars_container_node_type"
_STD_PARAM_NAME_CONTAINER_NODE_NS_PREFIX = "mars_container_node_ns_prefix"


class MARSTopologyLauncher:
    """Topology parser and mars_topolgy_edge and _vertex launcher.

    This class manages the parsing of  for e.g. an OpentTCS topology file and
    launches the edges and vertices as individual ros nodes of the type:
    ars_topology_edge or mars_topology_vertex.

    Attributes:
        __nh: Node handle zum lesen globaler paramter.
        __nh_priv: Node handle zum lesen privater parameter.
        __log_level: Log level of the ros console as a string.
        __topology_file_path: Path to the topology file.
        __topology_file_type: Type of the file which should be read.
        __topology_parser: Parser for parsing a topology file. A parser
                            always creates the mars topology entities
                            (vertex and edge) which can be started as an
                            topology agent.
        __mtl_vertex_node_pkg: Descripes the package name of a node.
        __mtl_vertex_node_type: Descripes file of a node which has to be launched.
        __mtl_vertex_node_ns_prefix: Descripes the namespace prefix of a node.
            MARS common prefix is: global_prefix + sub_prefix + node_id
            Global prefix for e.g.: mars/topology/
            Sub prefix for e.g. is the type: vertex/
            Note: the node_id is automatically added to the namespace
            Complete namespace: mars/topology/vertex/id
        __mtl_edge_node_pkg: Descripes the package name of a node.
        __mtl_edge_node_type: Descripes file of a node which has to be launched.
        __mtl_edge_node_ns_prefix: Descripes the namespace prefix of a node.
            MARS common prefix is: global_prefix + sub_prefix + node_id
            Global prefix for e.g.: mars/topology/
            Sub prefix for e.g. is the type: edge/
            Note: the node_id is automatically added to the namespace
            Complete namespace: mars/topology/edge/id
    """

    def __init__(self):
        self.__nh = ""
        self.__nh_priv = "~"
        self.__log_level = _STD_PARAM_LOG_LEVEL
        self.__topology_file_path = ""
        self.__topology_file_type = _TOPOLOGY_FILE_TYPE_OPENTCS
        self.__topology_topic_name = _TOPOLOGY_TOPIC_NAME
        self.__topology_parser = None
        self.__topology_launcher = MARSNodeLauncher()
        self.__topology_container_launcher = MARSContainerLauncher.MARSContainerLauncher()
        self.__topology_container_launcher.set_split_mode_random(
            MARSContainerLauncher._CONTAINER_SPLIT_MODE_RANDOM_ENTITY_COUNT_INFINITE)
        self.__mtl_topology_launch_mode = None

        # Parameter for MARSNodeLauncher (Parameter generic)
        self.__mtl_entity_max_linear_velocity = None
        self.__mtl_entity_max_angular_velocity = None
        self.__mtl_entity_max_linear_acceleration = None
        self.__mtl_entity_max_angular_acceleration = None

        # Parameter for MARSNodeLauncher (Parameter vertex)
        self.__mtl_vertex_node_pkg = ""
        self.__mtl_vertex_node_type = ""
        self.__mtl_vertex_node_ns_prefix = ""

        self.__mtl_vertex_footprint_bb_type = _STD_PARAM_VERTEX_FOOTPRINT_BB_TYPE
        self.__mtl_vertex_footprint_radius = None
        self.__mtl_vertex_footprint_resolution = None

        # Parameter for MARSNodeLauncher (Parameter edge)
        self.__mtl_edge_node_pkg = ""
        self.__mtl_edge_node_type = ""
        self.__mtl_edge_node_ns_prefix = ""

        # Parameter for MARSContainerLauncher (Parameter container)
        self.__mtl_container_node_pkg = ""
        self.__mtl_container_node_type = ""
        self.__mtl_container_node_ns_prefix = ""

    def __topology_launcher_pipeline(self):
        """Pipeline of the topology launcher node.

        Args:

        Returns:

        Raises:
        """
        successfully_created_topology = True

        if not self.__create_topology_parser():
            successfully_created_topology = False

        if successfully_created_topology:
            if (self.__topology_file_type == _TOPOLOGY_FILE_TYPE_OPENTCS):
                successfully_created_topology = self.__topology_parser.parse_file(
                    self.__topology_file_path)
            elif (self.__topology_file_type == _TOPOLOGY_FILE_TYPE_OPIL_SP):
                successfully_created_topology = self.__topology_parser.parse_file(
                    self.__topology_topic_name)
            elif (self.__topology_file_type == _TOPOLOGY_FILE_TYPE_GRAPHML):
                successfully_created_topology = self.__topology_parser.parse_file(
                    self.__topology_file_path)
            else:
                rospy.logerr("[MARSTopologyLauncher][__topology_launcher_pipeline]"
                             " Unknown topology type was given.")
        else:
            rospy.logerr("[MARSTopologyLauncher][__topology_launcher_pipeline]"
                         " Can't create topology parser.")

        if successfully_created_topology:

            if (self.__mtl_topology_launch_mode == _TOPOLOGY_LAUNCH_MODE_AGENT):

                # launch vertices
                self.__topology_launcher.launch_mars_vertices(
                    self.__topology_parser.get_mars_topology_vertices(),
                    self.__mtl_vertex_node_pkg,
                    self.__mtl_vertex_node_type,
                    self.__mtl_vertex_node_ns_prefix,
                    self.__mtl_entity_max_linear_velocity,
                    self.__mtl_entity_max_angular_velocity,
                    self.__mtl_entity_max_linear_acceleration,
                    self.__mtl_entity_max_angular_acceleration)

                # launch edges
                self.__topology_launcher.launch_mars_edges(
                    self.__topology_parser.get_mars_topology_edges(),
                    self.__mtl_edge_node_pkg,
                    self.__mtl_edge_node_type,
                    self.__mtl_edge_node_ns_prefix,
                    self.__mtl_entity_max_linear_velocity,
                    self.__mtl_entity_max_angular_velocity,
                    self.__mtl_entity_max_linear_acceleration,
                    self.__mtl_entity_max_angular_acceleration)
            elif (self.__mtl_topology_launch_mode == _TOPOLOGY_LAUNCH_MODE_CONTAINER):
                self.__topology_container_launcher.launch_container(
                    self.__topology_parser.get_mars_topology_vertices(),
                    self.__topology_parser.get_mars_topology_edges(),
                    self.__mtl_container_node_pkg,
                    self.__mtl_container_node_type,
                    self.__mtl_container_node_ns_prefix,
                    self.__mtl_entity_max_linear_velocity,
                    self.__mtl_entity_max_angular_velocity,
                    self.__mtl_entity_max_linear_acceleration,
                    self.__mtl_entity_max_angular_acceleration)
            else:
                rospy.logerr("[MARSTopologyLauncher][__topology_launcher_pipeline]"
                             " Unknown " + _STD_PARAM_NAME_TOPOLOGY_LAUNCH_MODE + " set: " + self.
                             __mtl_topology_launch_mode + ". Supported are: " +
                             _TOPOLOGY_LAUNCH_MODE_AGENT + " and " + _TOPOLOGY_LAUNCH_MODE_CONTAINER)
        else:
            rospy.logerr("[MARSTopologyLauncher][__topology_launcher_pipeline]"
                         " Can't create topology parser.")

        return successfully_created_topology

    def __create_topology_parser(self):
        successfully_created__topology_parser = True

        if self.__topology_file_type == _TOPOLOGY_FILE_TYPE_OPENTCS:
            self.__topology_parser = OpenTCSParser(
                footprint_type=self.__mtl_vertex_footprint_bb_type,
                footprint_radius=self.__mtl_vertex_footprint_radius,
                footprint_resolution=self.__mtl_vertex_footprint_resolution)
        elif self.__topology_file_type == _TOPOLOGY_FILE_TYPE_OPIL_SP:
            if (_OPIL_PARSER_AVAILABLE):
                self.__topology_parser = OpilSPParser(
                    footprint_type=MARSFootprintType.MARS_FOOTPRINT_TYPE_SQUARE,
                    footprint_radius=self.__mtl_vertex_footprint_radius,
                    footprint_resolution=self.__mtl_vertex_footprint_resolution)
            else:
                rospy.logerr(
                    "[MARSTopologyLauncher][__create__topology_parser]"
                    " Can't create opil parse, msg are not available!")
                successfully_created__topology_parser = False
        elif self.__topology_file_type == _TOPOLOGY_FILE_TYPE_GRAPHML:
            self.__topology_parser = GrapMLParser(
                footprint_type=self.__mtl_vertex_footprint_bb_type,
                footprint_radius=self.__mtl_vertex_footprint_radius,
                footprint_resolution=self.__mtl_vertex_footprint_resolution)
        else:
            rospy.logerr(
                "[MARSTopologyLauncher][__create__topology_parser]"
                " Unsupported topology parser type set: " + self.__topology_file_type +
                " Supported topology file types are: " + _TOPOLOGY_FILE_TYPE_OPENTCS + ", " +
                _TOPOLOGY_FILE_TYPE_OPIL_SP + ", " + _TOPOLOGY_FILE_TYPE_GRAPHML)
            successfully_created__topology_parser = False

        return successfully_created__topology_parser

    def run_ros_node(self):
        """Runs the ros node.

        Runs the ros node. This is the only method you have to call.

        Args:

        Returns:
            Returns true if now error occurred during runtime.

        Raises:
        """
        run_node_successfully = True

        if self.__init():
            rospy.loginfo(
                "[MARSTopologyLauncher][_ros_main_loop] Node successfully "
                "initialized. Starting pipeline!")

            try:
                # Pipeline to parse topology and start topology agents
                self.__topology_launcher_pipeline()

                self.__ros_main_loop()
            except NodeLaunchException as err:
                rospy.logerr(str(err))
                rospy.logerr("[MARSTopologyLauncher][run_ros_node] Shutting down node!")
        else:
            run_node_successfully = False

        return run_node_successfully

    def __get_param(self, nh, param_name, default_value=None):
        """Reads the parameter and print the result.

        Reads the parameter from the paramter server and print the result. If no
        value can be find on the parameter server, a default value will
        be assigned.

        Args:
            nh: node handle, global or private.
            param_name: name of the parameter.
            default_value: value which has to be assigned if no parameter
                can be found on the parameter server. (optional)

        Returns:
            Returns the value of the requested ros param.

        Raises:
            Raises ReadParamException if no value can be found on the paramter
                server and no default value was given.
        """
        value = default_value

        if default_value == None:
            if nh == "" or nh == "~":
                if rospy.has_param(nh + param_name):
                    value = rospy.get_param(nh + param_name)
                    rospy.loginfo("[MARSTopologyLauncher][__get_param] "
                                  + "Found parameter: "
                                  + str(param_name) + ", value: " + str(value))
                else:
                    error = "[MARSTopologyLauncher][__get_param] Could not get " \
                        + "param: " + param_name
                    raise ReadParamException(error)
            else:
                error = "[MARSTopologyLauncher][__get_param] " \
                    + "Unsupported node handle given: " + str(nh) \
                    + " Only empty string or '~' is allowed! Assigning" \
                    + " default vaulue!"
                raise ReadParamException(error)
        else:
            if nh == "" or nh == "~":
                if rospy.has_param(nh + param_name):
                    value = rospy.get_param(nh + param_name)
                    rospy.loginfo("[MARSTopologyLauncher][__get_param] "
                                  + "Found parameter: "
                                  + str(param_name) + ", value: " + str(value))
                else:
                    rospy.loginfo("[MARSTopologyLauncher][__get_param] "
                                  + "Cannot find value for parameter: "
                                  + str(param_name) + ", assigning default: "
                                  + str(default_value))
            else:
                rospy.logerr("[MARSTopologyLauncher][__get_param] "
                             + "Unsupported node handle given: " + str(nh)
                             + " Only empty string or '~' is allowed!"
                             + " Assigning default vaulue!")

        return value

    def __init(self):
        """Inits the node.

        This method automatically calls the methods: initServices(),
        initPublisher(), initSubsriber(), readLaunchParams()
        and printNodeInfos() if no error occours during execution.

        Args:

        Returns:
            Returns true if no error occours.

        Raises:
        """
        init_successfully = True

        # first read launch parameter
        init_successfully = self.__read_launch_params()

        return init_successfully

    def __read_launch_params(self):
        """Reads the node paramters.

        Reads the paramters from the launch file or "directly" from the
        parameter sever. In this method only private parameter will be readed.

        Args:

        Returns:
            Returns true if no error occours.

        Raises:
        """
        # IF a needed parameter is not provided,
        # set 'init_successfully' to 'False'!
        init_successfully = True

        # read node params with default value
        self.__log_level = self.__get_param(
            self.__nh_priv, _STD_PARAM_NAME_LOG_LEVEL, _STD_PARAM_LOG_LEVEL)
        # set node log level
        self.__set_node__log_level()

        self.__topology_file_type = self.__get_param(
            self.__nh_priv, _STD_PARAM_NAME_TOPOLOGY_FILE_TYPE,
            _STD_PARAM_TOPOLOGY_FILE_TYPE)

        self.__topology_topic_name = self.__get_param(
            self.__nh_priv, _STD_PARAM_NAME_TOPO_TOPIC_NAME,
            _STD_PARAM_TOPO_TOPIC_NAME)

        self.__mtl_vertex_footprint_bb_type = MARSFootprintType(self.__get_param(
            self.__nh_priv, _STD_PARAM_NAME_VERTEX_FOOTPRINT_BB_TYPE,
            _STD_PARAM_VERTEX_FOOTPRINT_BB_TYPE))

        self.__mtl_topology_launch_mode = self.__get_param(
            self.__nh_priv, _STD_PARAM_NAME_TOPOLOGY_LAUNCH_MODE,
            _STD_PARAM_TOPOLOGY_LAUNCH_MODE)

        # read node params without default value
        try:
            # Check here for file types which needs a file on the system
            if ((self.__topology_file_type == _TOPOLOGY_FILE_TYPE_OPENTCS)
                    or (self.__topology_file_type == _TOPOLOGY_FILE_TYPE_GRAPHML)):
                self.__topology_file_path = self.__get_param(
                    self.__nh_priv, _STD_PARAM_NAME_TOPOLOGY_FILE_PATH)

            # generic settings
            self.__mtl_entity_max_linear_velocity = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_ENTITY_MAX_LINEAR_VELOCITY)
            self.__mtl_entity_max_angular_velocity = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_ENTITY_MAX_ANGULAR_VELOCITY)
            self.__mtl_entity_max_linear_acceleration = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_ENTITY_MAX_LINEAR_ACCELERATION)
            self.__mtl_entity_max_angular_acceleration = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_ENTITY_MAX_ANGULAR_ACCELERATION)

            # vertex settings
            self.__mtl_vertex_node_pkg = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_VERTEX_NODE_PKG)
            self.__mtl_vertex_node_type = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_VERTEX_NODE_TYPE)
            self.__mtl_vertex_node_ns_prefix = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_VERTEX_NODE_NS_PREFIX)
            self.__mtl_vertex_footprint_radius = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_VERTEX_FOOTPRINT_RADIUS)
            self.__mtl_vertex_footprint_resolution = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_VERTEX_FOOTPRINT_RESOLUTION)

            # edge settings
            self.__mtl_edge_node_pkg = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_EDGE_NODE_PKG)
            self.__mtl_edge_node_type = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_EDGE_NODE_TYPE)
            self.__mtl_edge_node_ns_prefix = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_EDGE_NODE_NS_PREFIX)

            # container settings
            self.__mtl_container_node_pkg = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_CONTAINER_NODE_PKG)
            self.__mtl_container_node_type = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_CONTAINER_NODE_TYPE)
            self.__mtl_container_node_ns_prefix = self.__get_param(
                self.__nh_priv, _STD_PARAM_NAME_CONTAINER_NODE_NS_PREFIX)

        except ReadParamException as err:
            rospy.logerr(str(err))
            init_successfully = False

        return init_successfully

    def __set_node__log_level(self):
        """Sets the ros log level.

        Sets the log level of the ros console for the node.

        Args:

        Returns:

        Raises:
        """
        log_level = None

        if (self.__log_level == _LOG_LEVEL_DEBUG):
            log_level = rospy.DEBUG
        elif (self.__log_level == _LOG_LEVEL_INFO):
            log_level = rospy.INFO
        elif (self.__log_level == _LOG_LEVEL_WARN):
            log_level = rospy.WARN
        elif (self.__log_level == _LOG_LEVEL_ERROR):
            log_level = rospy.ERROR
        elif(self.__log_level == _LOG_LEVEL_FATAL):
            log_level = rospy.FATAL
        else:
            rospy.loginfo("[MARSTopologyLauncher][__set_node__log_level]"
                          " Unsupported log level set: '" + str(self.__log_level) +
                          "' Set log level to info")

            log_level = rospy.INFO

        rospy.impl.rosout.load_rosout_handlers(log_level)

    def __ros_main_loop(self):
        rate = rospy.Rate(_NODE_RATE)

        while not rospy.is_shutdown():
            rate.sleep()


# main function
if __name__ == '__main__':
    rospy.init_node(_NODE_NAME)

    try:
        mars_topology_launcher = MARSTopologyLauncher()

        mars_topology_launcher.run_ros_node()
    # Exception will be thrown if ros node is killed via terminal
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(
            "[MARSTopologyLauncher][__main__] Unexpected error occured. MARSTopologyLauncher crashed: (" + str(e) + ")!")

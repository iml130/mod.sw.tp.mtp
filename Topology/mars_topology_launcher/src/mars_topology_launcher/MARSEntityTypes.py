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

from enum import Enum
from TopologyException import TopologyException
from mars_topology_msgs.msg import TopologyEntityType

import rospy


class MARSEntityTypes(Enum):
    # MARSEntity global type(s)
    TOPOLOGY_ENTITY_TYPE_UNKNOWN = TopologyEntityType.TOPOLOGY_ENTITY_TYPE_UNKNOWN

    # MARSVertex types
    TOPOLOGY_VERTEX_TYPE_WAYPOINT = TopologyEntityType.TOPOLOGY_VERTEX_TYPE_WAYPOINT
    TOPOLOGY_VERTEX_TYPE_PICKING_STATION = TopologyEntityType.TOPOLOGY_VERTEX_TYPE_PICKING_STATION
    TOPOLOGY_VERTEX_TYPE_PARKING_LOT = TopologyEntityType.TOPOLOGY_VERTEX_TYPE_PARKING_LOT

    # MARSEdge type(s)
    TOPOLOGY_EDGE_TYPE_EDGE = TopologyEntityType.TOPOLOGY_EDGE_TYPE_EDGE

    def map_type(self, type):
        """ Describes the format of a MARSEntity a Id should be created.

            Args: 
                type: Type of a MARSEntity (int).
                        See constants:  MARSEntityTypes.TOPOLOGY_ENTITY_TYPE_UNKNOWN, 
                                        MARSEntityTypes.TOPOLOGY_VERTEX_TYPE_WAYPOINT,
                                        MARSEntityTypes.TOPOLOGY_VERTEX_TYPE_PICKING_STATION,
                                        MARSEntityTypes.TOPOLOGY_VERTEX_TYPE_PARKING_LOT,
                                        MARSEntityTypes.TOPOLOGY_EDGE_TYPE_EDGE

            Returns:
                The detected type. If type was unknown, MARSEntityTypes.TOPOLOGY_ENTITY_TYPE_UNKNOWN 
                will be returned.
            Raises:
        """

        if isinstance(type, int):
            if type == self.TOPOLOGY_ENTITY_TYPE_UNKNOWN:
                return self.TOPOLOGY_ENTITY_TYPE_UNKNOWN
            elif type == self.TOPOLOGY_VERTEX_TYPE_WAYPOINT:
                return self.TOPOLOGY_VERTEX_TYPE_WAYPOINT
            elif type == self.TOPOLOGY_VERTEX_TYPE_PICKING_STATION:
                return self.TOPOLOGY_VERTEX_TYPE_PICKING_STATION
            elif type == self.TOPOLOGY_VERTEX_TYPE_PARKING_LOT:
                return self.TOPOLOGY_VERTEX_TYPE_PARKING_LOT
            elif type == self.TOPOLOGY_EDGE_TYPE_EDGE:
                return self.TOPOLOGY_EDGE_TYPE_EDGE
            else:
                rospy.logwarn("[MARSEntityTypes][map_type] Can't map type, "
                              "type with value '" + str(type) + "' unknown!"
                              " Returning 'TOPOLOGY_ENTITY_TYPE_UNKNOWN'")
                return self.TOPOLOGY_ENTITY_TYPE_UNKNOWN
        else:
            rospy.logwarn("[MARSEntityTypes][map_type] Can't map type, "
                          "type has unsupported type. Only 'int' is supported!"
                          " Returning 'TOPOLOGY_ENTITY_TYPE_UNKNOWN'")
            return self.TOPOLOGY_ENTITY_TYPE_UNKNOWN

    def get_name(self):
        return self._name_

    def get_value(self):
        return self._value_

    def __str__(self):
        return str(self._name_) + "(" + str(self._value_) + ")"

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

from mars_common.Id import Id, IdType
from abc import ABCMeta, abstractmethod
from MARSEntityTypes import MARSEntityTypes

import rospy

_DEFAULT_ENTITY_MAXIMUM_HEIGHT = 2.0  # 2m
_DEFAULT_ENTITY_MAXIMUM_TOTAL_WEIGHT = 200.0  # 200 kg


class MARSEntity():
    """A mars entity represents a topology entity of the multi agent 
    routing service.

    A mars entity represents a topology entity of the multi agent 
    routing service. An entity can be a vertex or an edge.

    Attributes:
        _id: Unique ID of an mars agent / entity.
        _x_position: X-Position of the entity.
        _y_position: Y-Position of the entity.
        _type: Type of the mars entity (MARSEntityType)
        _is_locked: Describes whether a mars entity is locked or not (bool).
    """

    __metaclass__ = ABCMeta

    def __init__(self, id, x_position=0.0, y_position=0.0,
                 type=MARSEntityTypes.TOPOLOGY_ENTITY_TYPE_UNKNOWN,
                 is_locked=False,
                 forbidden_hazard_types=list(),
                 forbidden_vehicle_types=list(),
                 maximum_height=_DEFAULT_ENTITY_MAXIMUM_HEIGHT,
                 maximum_total_weight=_DEFAULT_ENTITY_MAXIMUM_TOTAL_WEIGHT,
                 max_linear_velocity=None,
                 max_angular_velocity=None,
                 max_linear_acceleration=None,
                 max_angular_acceleration=None, 
                 x_footprint = None, 
                 y_footprint= None):
        """Creates an object of type MARSEntity.

        Creates an object of type MARSEntity and sets an unique id! 

        Args:
            id: ID of the vertex. The ID must be unique and from type Id! 
            x_position: X-Position of the mars topology entity in meter [m].
            y_position: Y-Position of the mars topology entity in meter [m].
            type: Represents the type of the current mars topology entity 
                (MARSEntityType).
            is_locked: Describes whether a mars entity is locked or not (bool).

        Returns:

        Raises:
        """
        if isinstance(id, Id):
            self._id = id
            self._x_position = x_position
            self._y_position = y_position
            self._is_locked = is_locked
            self._forbidden_hazard_types = forbidden_hazard_types
            self._forbidden_vehicle_types = forbidden_vehicle_types
            self._maximum_height = maximum_height
            self._maximum_total_weight = maximum_total_weight
            self._max_linear_velocity = max_linear_velocity
            self._max_angular_velocity = max_angular_velocity
            self._max_linear_acceleration = max_linear_acceleration
            self._max_angular_acceleration = max_angular_acceleration
            self._footprint_x = x_footprint
            self._footprint_y = y_footprint

            self.set_type(type)
        else:
            raise TypeError(
                "[MARSEntity][MARSEntity] The given id was not from type Id!")

    def get_id(self):
        return self._id

    def set_name(self, name):
        self._id.set_description(name)

    def get_name(self):
        return self._id.get_description()

    def set_position(self, x_position, y_position):
        """Sets the position of the MARSEntity.

        Sets the position of the MARSEntity. The position is in meter!

        Args:
            x_position: X-Position of the mars topology entity in meter [m].
            y_position: Y-Position of the mars topology entity in meter [m].

        Returns:

        Raises:
        """
        self._x_position = x_position
        self._y_position = y_position

    def get_x_position(self):
        """Returns the x-position of the MARSEntity.

        Returns the position of the MARSEntity. The position is in meter!

        Args:

        Returns:
            Returns the x-position of the MARSEntity in meter [m].

        Raises:
        """
        return self._x_position

    def get_y_position(self):
        """Returns the y-position of the MARSEntity.

        Returns the position of the MARSEntity. The position is in meter!

        Args:

        Returns:
            Returns the y-position of the MARSEntity in meter [m].

        Raises:
        """
        return self._y_position

    def set_type(self, type_value):
        try:
            self._type = MARSEntityTypes(type_value)
        except:
            rospy.logwarn("[MARSEntity][set_type] Unknown type. "
                          "Type must be from type 'MARSEntityTypes'!")

    def get_type(self):
        return self._type

    def set_lock(self, is_locked):
        self._is_locked = is_locked

    def get_lock(self):
        return self._is_locked

    def set_forbidden_hazard_types(self, forbidden_hazard_types):
        self._forbidden_hazard_types = forbidden_hazard_types

    def get_forbidden_hazard_types(self):
        return self._forbidden_hazard_types

    def set_forbidden_vehicle_types(self, forbidden_vehicle_types):
        self._forbidden_vehicle_types = forbidden_vehicle_types

    def get_forbidden_vehicle_types(self):
        return self._forbidden_vehicle_types

    def set_maximum_height(self, maximum_height):
        self._maximum_height = maximum_height

    def get_maximum_height(self):
        return self._maximum_height

    def set_maximum_total_weight(self, maximum_total_weight):
        self._maximum_total_weight = maximum_total_weight

    def get_maximum_total_weight(self):
        return self._maximum_total_weight

    def set_max_linear_velocity(self, max_linear_velocity):
        self._max_linear_velocity = max_linear_velocity

    def get_max_linear_velocity(self):
        return self._max_linear_velocity

    def set_max_angular_velocity(self, max_angular_velocity):
        self._max_angular_velocity = max_angular_velocity

    def get_max_angular_velocity(self):
        return self._max_angular_velocity

    def set_max_linear_acceleration(self, max_linear_acceleration):
        self._max_linear_acceleration = max_linear_acceleration

    def get_max_linear_acceleration(self):
        return self._max_linear_acceleration

    def set_max_angular_acceleration(self, max_angular_acceleration):
        self._max_angular_acceleration = max_angular_acceleration

    def get_max_angular_acceleration(self):
        return self._max_angular_acceleration

    def add_footprint(self, x_footprint, y_footprint):
        self._footprint_x = x_footprint
        self._footprint_y = y_footprint

    def get_footprintX(self):
        return self._footprint_x

    def get_footprintY(self):
        return self._footprint_y

    @abstractmethod
    def __str__(self):
        pass

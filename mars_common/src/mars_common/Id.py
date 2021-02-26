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

import sys
import uuid
from enum import Enum, unique

import rospy
import mars_common_msgs.msg


class IdType(Enum):
    """ Describes the format from which a Id should be created.

        Args:

        Returns:

        Raises:
    """

    ID_TYPE_UNKNOWN = 0
    ID_TYPE_STRING_UUID = 10
    ID_TYPE_STRING_NAME = 20
    ID_TYPE_BYTE_ARRAY = 30
    ID_TYPE_PYTHON_UUID = 40

    def __str__(self):
        return str(self._name_) + "(" + str(self._value_) + ")"


class Id():

    UUID_HEXDEC_NOT_SPLITTED_LENGTH = 32
    UUID_HEXDEC_SPLITTED_LENGTH = 36

    def __init__(self, id, id_type, description=""):

        try:

            tmp_id = IdType(id_type)

            if (IdType.ID_TYPE_STRING_NAME == tmp_id):
                self.__id = uuid.uuid5(uuid.NAMESPACE_DNS, str(id))
            elif (IdType.ID_TYPE_BYTE_ARRAY == tmp_id):
                self.__id = uuid.UUID(bytes=id)
            elif (IdType.ID_TYPE_PYTHON_UUID == tmp_id):
                self.__id = id
            elif (IdType.ID_TYPE_STRING_UUID == tmp_id):

                if (len(id) == self.UUID_HEXDEC_NOT_SPLITTED_LENGTH):
                    self.__id = uuid.UUID(id)
                elif (len(id) == self.UUID_HEXDEC_SPLITTED_LENGTH):
                    self.__id = uuid.UUID("{" + id + "}")
                else:
                    raise TypeError("[Id][Id] Given id (uuid format) has wrong length. "
                                    "Length was " + len(id) + " but must be "
                                    + self.UUID_HEXDEC_NOT_SPLITTED_LENGTH
                                    + " or " + self.UUID_HEXDEC_SPLITTED_LENGTH)
            else:
                raise TypeError("[Id][Id] Given type was "
                                "IdType.ID_TYPE_UNKNOWN which is not allowed.")

            self.__description = description

        except Exception as ex:
            rospy.logwarn("[Id][Id] Unknown type. "
                          "Type can not be mapped with IdTypes!")
            raise TypeError("[Id][Id] Unknown type. "
                            "Type can not be mapped with IdTypes! (" + str(ex) + ")")

    def set_id(self, id):
        if isinstance(id, Id):
            self.__id = id
        else:
            raise TypeError("[Id][set_id] Given id was not in format Id!")

    def get_id(self):
        return self.__id

    def set_description(self, description):
        self.__description = description

    def get_description(self):
        return self.__description

    def to_msg(self):
        id_msg_array = []

        for x in self.__id.bytes:
            if(sys.version_info.major == 2):
                id_msg_array.append(int(x.encode('hex'), 16))
            elif(sys.version_info.major == 3):
                id_msg_array.append(int(x))

        return mars_common_msgs.msg.Id(uuid=id_msg_array, description=self.__description)

    def __eq__(self, other):
        return self.__id == other.get_id()

    def __str__(self):
        return ("id: " + str(self.__id)
                + ", description: " + str(self.__description))

    def __repr__(self):
        return ("id: " + str(self.__id)
                + ", description: " + str(self.__description))

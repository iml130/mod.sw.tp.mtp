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


class MARSPath:
    def __init__(self, origin_vertex_id, dst_vertex_id, max_velocity):
        self.__origin_vertex_id = origin_vertex_id
        self.__dst_vertex_id = dst_vertex_id
        self.__max_velocity = max_velocity

    def get_origin_vertex_id(self):
        return self.__origin_vertex_id

    def get_dst_vertex_id(self):
        return self.__dst_vertex_id

    def get_max_velocity(self):
        return self.__max_velocity
    
    def __str__(self):
        # orogin: origin_id (origin_desciption) --> dst: dst_id (dst_desciption)        
        # origin: 1 (Point-1) --> dst: 2 (Point-2)
        return ("origin: " + str(self.__origin_vertex_id.get_id()) 
                + " (" + str(self.__origin_vertex_id.get_description()) + ") "
                + " --> dst: " + str(self.__dst_vertex_id.get_id()) 
                + " (" + str(self.__dst_vertex_id.get_description()) + ")")
    
    def __repr__(self):
        # orogin: origin_id (origin_desciption) --> dst: dst_id (dst_desciption)        
        # origin: 1 (Point-1) --> dst: 2 (Point-2)
        return ("origin: " + str(self.__origin_vertex_id.get_id()) 
                + " (" + str(self.__origin_vertex_id.get_description()) + ") "
                + " --> dst: " + str(self.__dst_vertex_id.get_id()) 
                + " (" + str(self.__dst_vertex_id.get_description()) + ")")


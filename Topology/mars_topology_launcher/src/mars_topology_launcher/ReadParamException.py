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

class ReadParamException(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(str(self.value))
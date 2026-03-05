# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_project:msg/WaypointList.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'latitudes'
# Member 'longitudes'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WaypointList(type):
    """Metaclass of message 'WaypointList'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rover_project')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rover_project.msg.WaypointList')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__waypoint_list
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__waypoint_list
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__waypoint_list
            cls._TYPE_SUPPORT = module.type_support_msg__msg__waypoint_list
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__waypoint_list

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WaypointList(metaclass=Metaclass_WaypointList):
    """Message class 'WaypointList'."""

    __slots__ = [
        '_latitudes',
        '_longitudes',
        '_total_waypoints',
        '_current_index',
    ]

    _fields_and_field_types = {
        'latitudes': 'sequence<double>',
        'longitudes': 'sequence<double>',
        'total_waypoints': 'uint32',
        'current_index': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.latitudes = array.array('d', kwargs.get('latitudes', []))
        self.longitudes = array.array('d', kwargs.get('longitudes', []))
        self.total_waypoints = kwargs.get('total_waypoints', int())
        self.current_index = kwargs.get('current_index', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.latitudes != other.latitudes:
            return False
        if self.longitudes != other.longitudes:
            return False
        if self.total_waypoints != other.total_waypoints:
            return False
        if self.current_index != other.current_index:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def latitudes(self):
        """Message field 'latitudes'."""
        return self._latitudes

    @latitudes.setter
    def latitudes(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'latitudes' array.array() must have the type code of 'd'"
            self._latitudes = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'latitudes' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._latitudes = array.array('d', value)

    @builtins.property
    def longitudes(self):
        """Message field 'longitudes'."""
        return self._longitudes

    @longitudes.setter
    def longitudes(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'longitudes' array.array() must have the type code of 'd'"
            self._longitudes = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'longitudes' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._longitudes = array.array('d', value)

    @builtins.property
    def total_waypoints(self):
        """Message field 'total_waypoints'."""
        return self._total_waypoints

    @total_waypoints.setter
    def total_waypoints(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'total_waypoints' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'total_waypoints' field must be an unsigned integer in [0, 4294967295]"
        self._total_waypoints = value

    @builtins.property
    def current_index(self):
        """Message field 'current_index'."""
        return self._current_index

    @current_index.setter
    def current_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_index' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'current_index' field must be an unsigned integer in [0, 4294967295]"
        self._current_index = value

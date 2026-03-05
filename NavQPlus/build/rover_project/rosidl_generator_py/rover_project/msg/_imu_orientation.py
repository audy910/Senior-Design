# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_project:msg/ImuOrientation.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ImuOrientation(type):
    """Metaclass of message 'ImuOrientation'."""

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
                'rover_project.msg.ImuOrientation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__imu_orientation
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__imu_orientation
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__imu_orientation
            cls._TYPE_SUPPORT = module.type_support_msg__msg__imu_orientation
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__imu_orientation

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ImuOrientation(metaclass=Metaclass_ImuOrientation):
    """Message class 'ImuOrientation'."""

    __slots__ = [
        '_heading',
        '_pitch',
        '_roll',
        '_cal_sys',
        '_cal_mag',
    ]

    _fields_and_field_types = {
        'heading': 'float',
        'pitch': 'float',
        'roll': 'float',
        'cal_sys': 'uint8',
        'cal_mag': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.heading = kwargs.get('heading', float())
        self.pitch = kwargs.get('pitch', float())
        self.roll = kwargs.get('roll', float())
        self.cal_sys = kwargs.get('cal_sys', int())
        self.cal_mag = kwargs.get('cal_mag', int())

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
        if self.heading != other.heading:
            return False
        if self.pitch != other.pitch:
            return False
        if self.roll != other.roll:
            return False
        if self.cal_sys != other.cal_sys:
            return False
        if self.cal_mag != other.cal_mag:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def heading(self):
        """Message field 'heading'."""
        return self._heading

    @heading.setter
    def heading(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'heading' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'heading' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._heading = value

    @builtins.property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch = value

    @builtins.property
    def roll(self):
        """Message field 'roll'."""
        return self._roll

    @roll.setter
    def roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll = value

    @builtins.property
    def cal_sys(self):
        """Message field 'cal_sys'."""
        return self._cal_sys

    @cal_sys.setter
    def cal_sys(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cal_sys' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cal_sys' field must be an unsigned integer in [0, 255]"
        self._cal_sys = value

    @builtins.property
    def cal_mag(self):
        """Message field 'cal_mag'."""
        return self._cal_mag

    @cal_mag.setter
    def cal_mag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cal_mag' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cal_mag' field must be an unsigned integer in [0, 255]"
        self._cal_mag = value

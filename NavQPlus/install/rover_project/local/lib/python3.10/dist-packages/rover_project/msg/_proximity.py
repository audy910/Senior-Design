# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_project:msg/Proximity.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Proximity(type):
    """Metaclass of message 'Proximity'."""

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
                'rover_project.msg.Proximity')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__proximity
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__proximity
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__proximity
            cls._TYPE_SUPPORT = module.type_support_msg__msg__proximity
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__proximity

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Proximity(metaclass=Metaclass_Proximity):
    """Message class 'Proximity'."""

    __slots__ = [
        '_proximity_front',
        '_proximity_rear',
        '_proximity_cliff',
        '_cliff_detected',
        '_front_valid',
        '_rear_valid',
        '_cliff_valid',
    ]

    _fields_and_field_types = {
        'proximity_front': 'uint16',
        'proximity_rear': 'uint16',
        'proximity_cliff': 'uint16',
        'cliff_detected': 'boolean',
        'front_valid': 'boolean',
        'rear_valid': 'boolean',
        'cliff_valid': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.proximity_front = kwargs.get('proximity_front', int())
        self.proximity_rear = kwargs.get('proximity_rear', int())
        self.proximity_cliff = kwargs.get('proximity_cliff', int())
        self.cliff_detected = kwargs.get('cliff_detected', bool())
        self.front_valid = kwargs.get('front_valid', bool())
        self.rear_valid = kwargs.get('rear_valid', bool())
        self.cliff_valid = kwargs.get('cliff_valid', bool())

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
        if self.proximity_front != other.proximity_front:
            return False
        if self.proximity_rear != other.proximity_rear:
            return False
        if self.proximity_cliff != other.proximity_cliff:
            return False
        if self.cliff_detected != other.cliff_detected:
            return False
        if self.front_valid != other.front_valid:
            return False
        if self.rear_valid != other.rear_valid:
            return False
        if self.cliff_valid != other.cliff_valid:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def proximity_front(self):
        """Message field 'proximity_front'."""
        return self._proximity_front

    @proximity_front.setter
    def proximity_front(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'proximity_front' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'proximity_front' field must be an unsigned integer in [0, 65535]"
        self._proximity_front = value

    @builtins.property
    def proximity_rear(self):
        """Message field 'proximity_rear'."""
        return self._proximity_rear

    @proximity_rear.setter
    def proximity_rear(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'proximity_rear' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'proximity_rear' field must be an unsigned integer in [0, 65535]"
        self._proximity_rear = value

    @builtins.property
    def proximity_cliff(self):
        """Message field 'proximity_cliff'."""
        return self._proximity_cliff

    @proximity_cliff.setter
    def proximity_cliff(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'proximity_cliff' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'proximity_cliff' field must be an unsigned integer in [0, 65535]"
        self._proximity_cliff = value

    @builtins.property
    def cliff_detected(self):
        """Message field 'cliff_detected'."""
        return self._cliff_detected

    @cliff_detected.setter
    def cliff_detected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cliff_detected' field must be of type 'bool'"
        self._cliff_detected = value

    @builtins.property
    def front_valid(self):
        """Message field 'front_valid'."""
        return self._front_valid

    @front_valid.setter
    def front_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'front_valid' field must be of type 'bool'"
        self._front_valid = value

    @builtins.property
    def rear_valid(self):
        """Message field 'rear_valid'."""
        return self._rear_valid

    @rear_valid.setter
    def rear_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'rear_valid' field must be of type 'bool'"
        self._rear_valid = value

    @builtins.property
    def cliff_valid(self):
        """Message field 'cliff_valid'."""
        return self._cliff_valid

    @cliff_valid.setter
    def cliff_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cliff_valid' field must be of type 'bool'"
        self._cliff_valid = value

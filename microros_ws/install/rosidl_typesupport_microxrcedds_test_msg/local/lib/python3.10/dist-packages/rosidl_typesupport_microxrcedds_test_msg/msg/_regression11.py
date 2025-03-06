# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression11.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Regression11(type):
    """Metaclass of message 'Regression11'."""

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
            module = import_type_support('rosidl_typesupport_microxrcedds_test_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rosidl_typesupport_microxrcedds_test_msg.msg.Regression11')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__regression11
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__regression11
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__regression11
            cls._TYPE_SUPPORT = module.type_support_msg__msg__regression11
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__regression11

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Regression11(metaclass=Metaclass_Regression11):
    """Message class 'Regression11'."""

    __slots__ = [
        '_f',
        '_g',
        '_h',
        '_i',
        '_j',
        '_k',
        '_l',
    ]

    _fields_and_field_types = {
        'f': 'octet',
        'g': 'uint16',
        'h': 'octet',
        'i': 'boolean',
        'j': 'int32',
        'k': 'float',
        'l': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('octet'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('octet'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.f = kwargs.get('f', bytes([0]))
        self.g = kwargs.get('g', int())
        self.h = kwargs.get('h', bytes([0]))
        self.i = kwargs.get('i', bool())
        self.j = kwargs.get('j', int())
        self.k = kwargs.get('k', float())
        self.l = kwargs.get('l', int())

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
        if self.f != other.f:
            return False
        if self.g != other.g:
            return False
        if self.h != other.h:
            return False
        if self.i != other.i:
            return False
        if self.j != other.j:
            return False
        if self.k != other.k:
            return False
        if self.l != other.l:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def f(self):
        """Message field 'f'."""
        return self._f

    @f.setter
    def f(self, value):
        if __debug__:
            from collections.abc import ByteString
            assert \
                (isinstance(value, (bytes, ByteString)) and
                 len(value) == 1), \
                "The 'f' field must be of type 'bytes' or 'ByteString' with length 1"
        self._f = value

    @builtins.property
    def g(self):
        """Message field 'g'."""
        return self._g

    @g.setter
    def g(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'g' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'g' field must be an unsigned integer in [0, 65535]"
        self._g = value

    @builtins.property
    def h(self):
        """Message field 'h'."""
        return self._h

    @h.setter
    def h(self, value):
        if __debug__:
            from collections.abc import ByteString
            assert \
                (isinstance(value, (bytes, ByteString)) and
                 len(value) == 1), \
                "The 'h' field must be of type 'bytes' or 'ByteString' with length 1"
        self._h = value

    @builtins.property
    def i(self):
        """Message field 'i'."""
        return self._i

    @i.setter
    def i(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'i' field must be of type 'bool'"
        self._i = value

    @builtins.property
    def j(self):
        """Message field 'j'."""
        return self._j

    @j.setter
    def j(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'j' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'j' field must be an integer in [-2147483648, 2147483647]"
        self._j = value

    @builtins.property
    def k(self):
        """Message field 'k'."""
        return self._k

    @k.setter
    def k(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'k' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'k' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._k = value

    @builtins.property
    def l(self):
        """Message field 'l'."""
        return self._l

    @l.setter
    def l(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'l' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'l' field must be an unsigned integer in [0, 65535]"
        self._l = value
